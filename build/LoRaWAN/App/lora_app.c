/******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "sys_app.h"
#include "lora_app.h"
#include "stm32_seq.h"
#include "stm32_timer.h"
#include "utilities_def.h"
#include "app_version.h"
#include "lorawan_version.h"
#include "subghz_phy_version.h"
#include "lora_info.h"
#include "LmHandler.h"
#include "adc_if.h"
#include "CayenneLpp.h"
#include "sys_sensors.h"
#include "flash_if.h"

/* USER CODE BEGIN Includes */
#include "radio.h"
#include "stdbool.h"
#include "string.h"
#include "se-identity.h"
#include "usart.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "time.h"
#include "STM32_EEPROM.h"
#include "gpio.h"
#include "LmHandler.h"
#include "Macnman.h"
#include "i2c.h"
#include <stdlib.h>
#include <string.h>
#include "LIS2MDLSensor.h"
#include "lis2mdl.h"
#include "battery.h"
/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart2;
//extern ADC_HandleTypeDef hadc;

//For device uplink time randomness
bool is_critical_event_pending = false;
uint8_t event_retry_counter = 0;
bool is_retry_cycle = false;

#define MAX_EVENT_RETRIES 6

bool isTransmiting = false;
bool joining = true;
bool sendJoiningMsg = true;
bool waiting_for_first_ack = false;
uint8_t first_uplink_retry = 0;
uint8_t tamperType = 0;
uint32_t TX_DUTYCYCLE = 10;
uint32_t UPLINK_TIMER = 43200; // 12 Hours
uint8_t mode = 2;

// new variables
uint32_t SLEEP_TIME = 0;
bool tiltTamperDetected = false;
uint8_t tiltTamperCounter = 0;
uint8_t tamperBurstCounter = 0;   // Tracks how many burst messages we've sent
bool isPermanentSleep = false;    // The "Shutdown" flag
bool atEnabled = false;
bool joinStatus = false; //// LORAWAN JOIN STATUS ////
bool confirmUplink = false; //// CONFIRM UPLINK ////
bool uplinkStatus = false;
extern I2C_HandleTypeDef hi2c1;
int8_t rslt;
uint32_t ID = 0;
char txBuffer[100];

//float floatData[40];
const float multiply = 100.00;
uint8_t RaedflashData[4096];
uint8_t flash_address_index = 0;
uint32_t SendSize = 0;
bool Flash_read = false;

int8_t rslt;
uint8_t samplingCounter = 0;
float samplinData[36] = { 0 };
bool triggerUpState = false;
uint8_t triggerCount = 0;
SysTime_t lastUplinkTime = { 0 };
bool TimerState = false;

uint8_t dev_eui[8];
uint8_t join_eui[8];
uint8_t app_key[16];

int16_t x_initial_reading = 0;
int16_t y_initial_reading = 0;
int16_t z_initial_reading = 0;
bool sensorInit = false;

static uint16_t Parking_detection_counter = 0;
static uint16_t NoParking_detection_counter = 0;

SysTime_t parking_detection_start_time;
bool detection_timer_started = false;
bool last_uplink_parking_state = false;

#define MAX_QUEUE_SIZE 5

typedef struct {
	LmHandlerAppData_t AppData;
	bool IsTxConfirmed;
} UplinkQueueItem_t;

typedef struct {
	UplinkQueueItem_t queue[MAX_QUEUE_SIZE];
	int front;
	int rear;
	int count;
} UplinkQueue_t;

UplinkQueue_t uplinkQueue = { .front = 0, .rear = 0, .count = 0 };
//uint8_t Buffer[25] = { 0 };N1

//uint8_t Space[] = " - ";
//uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
//uint8_t EndMSG[] = "Done! \r\n\r\n";
/* USER CODE END EV */

/* Private typedef -----------------------------------------------------------*/
/**
 * @brief LoRa State Machine states
 */
typedef enum TxEventType_e {
	/**
	 * @brief Appdata Transmission issue based on timer every TxDutyCycleTime
	 */
	TX_ON_TIMER,
	/**
	 * @brief Appdata Transmission external event plugged on OnSendEvent( )
	 */
	TX_ON_EVENT
/* USER CODE BEGIN TxEventType_t */

/* USER CODE END TxEventType_t */
} TxEventType_t;

/* USER CODE BEGIN PTD */
//void MX_GPIO_Init(void);
void sendTamperData(void);
void sendResponceData(uint8_t state, uint8_t slaveId, uint8_t functionCode,
		uint16_t registerAddress, uint16_t registerValue);
uint16_t getBattVoltage(void);
void dataRate(uint8_t data_rate);
void lorawanClass(char class);
HAL_StatusTypeDef initilzeLorawanVariables(void);
LmHandlerErrorStatus_t requestDeviceTime(void);
void lis2mdl_parking_sensor(void);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/**
 * LEDs period value of the timer in ms
 */
#define LED_PERIOD_TIME 500

/**
 * Join switch period value of the timer in ms
 */
#define JOIN_TIME 2000

/*---------------------------------------------------------------------------*/
/*                             LoRaWAN NVM configuration                     */
/*---------------------------------------------------------------------------*/
/**
 * @brief LoRaWAN NVM Flash address
 * @note last 2 sector of a 128kBytes device
 */
#define LORAWAN_NVM_BASE_ADDRESS                    ((void *)0x0803F000UL)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private function prototypes -----------------------------------------------*/
/**
 * @brief  LoRa End Node send request
 */
static void SendTxData(void);

/**
 * @brief  TX timer callback function
 * @param  context ptr of timer context
 */
static void OnTxTimerEvent(void *context);

/**
 * @brief  join event callback function
 * @param  joinParams status of join
 */
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams);

/**
 * @brief callback when LoRaWAN application has sent a frame
 * @brief  tx event callback function
 * @param  params status of last Tx
 */
static void OnTxData(LmHandlerTxParams_t *params);

/**
 * @brief callback when LoRaWAN application has received a frame
 * @param appData data received in the last Rx
 * @param params status of last Rx
 */
static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params);

/**
 * @brief callback when LoRaWAN Beacon status is updated
 * @param params status of Last Beacon
 */
static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params);

/**
 * @brief callback when system time has been updated
 */
static void OnSysTimeUpdate(void);

/**
 * @brief callback when LoRaWAN application Class is changed
 * @param deviceClass new class
 */
static void OnClassChange(DeviceClass_t deviceClass);

/**
 * @brief  LoRa store context in Non Volatile Memory
 */
static void StoreContext(void);

/**
 * @brief  stop current LoRa execution to switch into non default Activation mode
 */
static void StopJoin(void);

/**
 * @brief  Join switch timer callback function
 * @param  context ptr of Join switch context
 */
static void OnStopJoinTimerEvent(void *context);

/**
 * @brief  Notifies the upper layer that the NVM context has changed
 * @param  state Indicates if we are storing (true) or restoring (false) the NVM context
 */
static void OnNvmDataChange(LmHandlerNvmContextStates_t state);

/**
 * @brief  Store the NVM Data context to the Flash
 * @param  nvm ptr on nvm structure
 * @param  nvm_size number of data bytes which were stored
 */
static void OnStoreContextRequest(void *nvm, uint32_t nvm_size);

/**
 * @brief  Restore the NVM Data context from the Flash
 * @param  nvm ptr on nvm structure
 * @param  nvm_size number of data bytes which were restored
 */
static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size);

/**
 * Will be called each time a Radio IRQ is handled by the MAC layer
 *
 */
static void OnMacProcessNotify(void);

/**
 * @brief Change the periodicity of the uplink frames
 * @param periodicity uplink frames period in ms
 * @note Compliance test protocol callbacks
 */
static void OnTxPeriodicityChanged(uint32_t periodicity);

/**
 * @brief Change the confirmation control of the uplink frames
 * @param isTxConfirmed Indicates if the uplink requires an acknowledgement
 * @note Compliance test protocol callbacks
 */
static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed);

/**
 * @brief Change the periodicity of the ping slot frames
 * @param pingSlotPeriodicity ping slot frames period in ms
 * @note Compliance test protocol callbacks
 */
static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity);

/**
 * @brief Will be called to reset the system
 * @note Compliance test protocol callbacks
 */
static void OnSystemReset(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
/**
 * @brief LoRaWAN default activation type
 */
static ActivationType_t ActivationType = LORAWAN_DEFAULT_ACTIVATION_TYPE;

/**
 * @brief LoRaWAN force rejoin even if the NVM context is restored
 */
static bool ForceRejoin = LORAWAN_FORCE_REJOIN_AT_BOOT;

/**
 * @brief LoRaWAN handler Callbacks
 */
static LmHandlerCallbacks_t LmHandlerCallbacks = { .GetBatteryLevel =
		GetBatteryLevel, .GetTemperature = GetTemperatureLevel, .GetUniqueId =
		GetUniqueId, .GetDevAddr = GetDevAddr, .OnRestoreContextRequest =
		OnRestoreContextRequest, .OnStoreContextRequest = OnStoreContextRequest,
		.OnMacProcess = OnMacProcessNotify, .OnNvmDataChange = OnNvmDataChange,
		.OnJoinRequest = OnJoinRequest, .OnTxData = OnTxData, .OnRxData =
				OnRxData, .OnBeaconStatusChange = OnBeaconStatusChange,
		.OnSysTimeUpdate = OnSysTimeUpdate, .OnClassChange = OnClassChange,
		.OnTxPeriodicityChanged = OnTxPeriodicityChanged,
		.OnTxFrameCtrlChanged = OnTxFrameCtrlChanged,
		.OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
		.OnSystemReset = OnSystemReset, };

/**
 * @brief LoRaWAN handler parameters
 */
static LmHandlerParams_t LmHandlerParams = { .ActiveRegion = ACTIVE_REGION,
		.DefaultClass = LORAWAN_DEFAULT_CLASS, .AdrEnable = LORAWAN_ADR_STATE,
		.IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE, .TxDatarate =
		LORAWAN_DEFAULT_DATA_RATE, .TxPower = LORAWAN_DEFAULT_TX_POWER,
		.PingSlotPeriodicity = LORAWAN_DEFAULT_PING_SLOT_PERIODICITY,
		.RxBCTimeout = LORAWAN_DEFAULT_CLASS_B_C_RESP_TIMEOUT };

/**
 * @brief Type of Event to generate application Tx
 */
static TxEventType_t EventType = TX_ON_TIMER;

// --- New Join Management Variables ---
uint8_t join_retry_counter = 0;
#define MAX_JOIN_ATTEMPTS 12
#define JOIN_BACKOFF_TIME (24 * 60 * 60 * 1000) // 24 hours time
static UTIL_TIMER_Object_t JoinBackoffTimer;
static void OnJoinBackoffTimerEvent(void *context); // Prototype for the backoff timer
/**
 * @brief Timer to handle the application Tx
 */
static UTIL_TIMER_Object_t TxTimer;

/**
 * @brief Tx Timer period
 */
static UTIL_TIMER_Time_t TxPeriodicity = APP_TX_DUTYCYCLE;

/**
 * @brief Join Timer period
 */
static UTIL_TIMER_Object_t StopJoinTimer;
/**
 * @brief Timer to handle the application Tx
 */

/* USER CODE BEGIN PV */

/**
 * @brief Timer to handle the application Tx Led to toggle
 */
static UTIL_TIMER_Object_t TxLedTimer;

/**
 * @brief Timer to handle the application Rx Led to toggle
 */
static UTIL_TIMER_Object_t RxLedTimer;

/**
 * @brief Timer to handle the application Join Led to toggle
 */
//static UTIL_TIMER_Object_t JoinLedTimer;
UTIL_TIMER_Object_t TriggerTimer;

UTIL_TIMER_Object_t flashReadTimer;
/**
 * @brief Join Timer period
 */
/*!
 * Package Application buffer
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/**
 * @brief  LED Tx timer callback function
 * @param  context ptr of LED context
 */
static void OnTxTimerLedEvent(void *context);

/**
 * @brief  LED Rx timer callback function
 * @param  context ptr of LED context
 */
static void OnRxTimerLedEvent(void *context);

/**
 * @brief  LED Join timer callback function
 * @param  context ptr of LED context
 */
static void OnJoinTimerLedEvent(void *context);

/*!
 * Package application data structure
 */
static LmHandlerAppData_t AppData = { 0, LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
		AppDataBuffer };

LmHandlerMsgTypes_t messageType = LORAMAC_HANDLER_CONFIRMED_MSG;

typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hour;
	uint8_t dayofweek;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} TIME;

TIME timet;

/* USER CODE END PV */

/* Exported functions ---------------------------------------------------------*/
/* USER CODE BEGIN EF */

/* USER CODE END EF */

void LoRaWAN_Init(void) {
	/* USER CODE BEGIN LoRaWAN_Init_LV */
	uint32_t feature_version = 0UL;
	MX_GPIO_Init();
	MX_I2C1_Init();

	/* USER CODE END LoRaWAN_Init_LV */

	/* USER CODE BEGIN LoRaWAN_Init_1 */

	/* Get LoRaWAN APP version*/
	APP_LOG(TS_OFF, VLEVEL_M, "APPLICATION_VERSION: V%X.%X.%X\n",
			(uint8_t)(APP_VERSION_MAIN), (uint8_t)(APP_VERSION_SUB1),
			(uint8_t)(APP_VERSION_SUB2));

	/* Get MW LoRaWAN info */
	APP_LOG(TS_OFF, VLEVEL_M, "MW_LORAWAN_VERSION:  V%X.%X.%X\n",
			(uint8_t)(LORAWAN_VERSION_MAIN), (uint8_t)(LORAWAN_VERSION_SUB1),
			(uint8_t)(LORAWAN_VERSION_SUB2));

	/* Get MW SubGhz_Phy info */
	APP_LOG(TS_OFF, VLEVEL_M, "MW_RADIO_VERSION:    V%X.%X.%X\n",
			(uint8_t)(SUBGHZ_PHY_VERSION_MAIN),
			(uint8_t)(SUBGHZ_PHY_VERSION_SUB1),
			(uint8_t)(SUBGHZ_PHY_VERSION_SUB2));

	/* Get LoRaWAN Link Layer info */
	LmHandlerGetVersion(LORAMAC_HANDLER_L2_VERSION, &feature_version);
	APP_LOG(TS_OFF, VLEVEL_M, "L2_SPEC_VERSION:     V%X.%X.%X\n",
			(uint8_t )(feature_version >> 24),
			(uint8_t )(feature_version >> 16), (uint8_t )(feature_version >> 8));

	/* Get LoRaWAN Regional Parameters info */
	LmHandlerGetVersion(LORAMAC_HANDLER_REGION_VERSION, &feature_version);
	APP_LOG(TS_OFF, VLEVEL_M, "RP_SPEC_VERSION:     V%X-%X.%X.%X\n",
			(uint8_t )(feature_version >> 24),
			(uint8_t )(feature_version >> 16), (uint8_t )(feature_version >> 8),
			(uint8_t )(feature_version));

	// Load region, class, ADR, data rate, TX interval from EEPROM
	initilzeLorawanVariables();


	if (FLASH_IF_Init(NULL) != FLASH_IF_OK) {
		Error_Handler();
	}

	/* USER CODE END LoRaWAN_Init_1 */

	UTIL_TIMER_Create(&StopJoinTimer, JOIN_TIME, UTIL_TIMER_ONESHOT,
			OnStopJoinTimerEvent, NULL);
	// Register sequencer tasks for main loop execution
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LmHandlerProcess), UTIL_SEQ_RFU,
			LmHandlerProcess);
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent),
	UTIL_SEQ_RFU, SendTxData);
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStoreContextEvent), UTIL_SEQ_RFU,
			StoreContext);
	UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), UTIL_SEQ_RFU,
			StopJoin);

	/* Init Info table used by LmHandler*/
	LoraInfo_Init();

	/* Init the Lora Stack*/
	LmHandlerInit(&LmHandlerCallbacks, APP_VERSION);

	LmHandlerConfigure(&LmHandlerParams);

	/* USER CODE BEGIN LoRaWAN_Init_2 */
	/* USER CODE END LoRaWAN_Init_2 */

	// Create JoinBackoffTimer BEFORE join (needed for retry delays)
	UTIL_TIMER_Create(&JoinBackoffTimer, JOIN_BACKOFF_TIME, UTIL_TIMER_ONESHOT,
			OnJoinBackoffTimerEvent, NULL);

	// Random boot delay (0-10s) to spread 5000 devices and prevent gateway flood
	uint32_t boot_delay = (Radio.Random() % 10000);
	APP_LOG(TS_OFF, VLEVEL_M, "Boot delay: %u ms\n", boot_delay);
	HAL_Delay(boot_delay);

	// Initiate OTAA Join — OnJoinRequest callback handles success/failure
	LmHandlerJoin(ActivationType, ForceRejoin);

	// Create TX timer (5s initial) and trigger timer (2s for downlink response)
	UTIL_TIMER_Create(&TxTimer, 5000, UTIL_TIMER_ONESHOT, OnTxTimerEvent, NULL);
	UTIL_TIMER_Create(&TriggerTimer, 2000, UTIL_TIMER_ONESHOT, OnTxTimerEvent,
	NULL);

	/* USER CODE BEGIN LoRaWAN_Init_Last */
	Battery_Init();
	/* USER CODE END LoRaWAN_Init_Last */
}

/* USER CODE BEGIN PB_Callbacks */

/* User should remove the #if 0 statement and adapt the below code according with his needs*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	/* 1. PERMANENT SLEEP CHECK */
    if (isPermanentSleep) {
    return;
    }
    static uint32_t lastTamperTrigger = 0;

    if (GPIO_Pin == TILT_SENSOR_Pin) {

    	 /* 2. BURST PROTECTION*/
        if (tamperBurstCounter > 0){
        return;
        }
        /* 3. SOFTWARE DEBOUNCING (2-Second Window)*/
        if ((HAL_GetTick() - lastTamperTrigger) > 5000) {
            tiltTamperCounter++;
        /* 4. TAMPER TRIGGER (3rd Confirmed Vibration)*/
            if (tiltTamperCounter >= 3) {
                tiltTamperDetected = true;
                tiltTamperCounter = 0;
                lastTamperTrigger = HAL_GetTick();
                UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
            }
        }
    }
}

/* USER CODE END PB_Callbacks */

/* Private functions ---------------------------------------------------------*/
/* USER CODE BEGIN PrFD */
static void OnJoinBackoffTimerEvent(void *context) {
    if (join_retry_counter >= MAX_JOIN_ATTEMPTS) {
        // 24-hour backoff expired — start fresh
        APP_LOG(TS_OFF, VLEVEL_M, "### 24h backoff over. Restarting Join...\n");
        join_retry_counter = 0;
    } else {
        // Short retry (2-8s) — don't reset counter
        APP_LOG(TS_OFF, VLEVEL_M, "### Join retry attempt %d...\n",
                join_retry_counter + 1);
    }
    LmHandlerJoin(ActivationType, ForceRejoin);
}


void sendTamperData(void) {
	uint8_t i = 0;
	UTIL_TIMER_Time_t nextTxIn = 0;
	if (LmHandlerDeviceTimeReq() == LORAMAC_HANDLER_SUCCESS) { // Time Synchronization
		APP_LOG(TS_ON, VLEVEL_M, "Time request succsess\r\n");
	}
	AppData.Port = LORAWAN_USER_APP_PORT;
	AppData.Buffer[i++] = 0;
	AppData.Buffer[i++] = 0x0A;  // Device Identification Byte

	// OEM_ID 1
	uint32_t val = LL_FLASH_GetSTCompanyID();
	AppData.Buffer[i++] = (val >> 16) & 0xFF;
	AppData.Buffer[i++] = (val >> 8) & 0xFF;
	AppData.Buffer[i++] = val & 0xFF;
	val = LL_FLASH_GetDeviceID();
	AppData.Buffer[i++] = val & 0xFF;

	// Hardware version
	AppData.Buffer[i++] = (uint8_t) (HARDWARE_VERSION_MAIN);
	AppData.Buffer[i++] = (uint8_t) (HARDWARE_VERSION_SUB1);
	AppData.Buffer[i++] = (uint8_t) (HARDWARE_VERSION_SUB2);

	// Software version
	AppData.Buffer[i++] = (uint8_t) (SOFTWARE_VERSION_MAIN);
	AppData.Buffer[i++] = (uint8_t) (SOFTWARE_VERSION_SUB1);
	AppData.Buffer[i++] = (uint8_t) (SOFTWARE_VERSION_SUB2);

	// uplink time interval
	AppData.Buffer[i++] = (uint8_t) (((TX_DUTYCYCLE) >> 8) & 0xFF); // TDCM
	AppData.Buffer[i++] = (uint8_t) ((TX_DUTYCYCLE) & 0xFF); 	// TDCM

	// time stamp payload
	SysTime_t UnixEpoch = SysTimeGet();
	UnixEpoch.Seconds -= 18; /*removing leap seconds*/
	UnixEpoch.Seconds += 19800; /*adding 5:30 hours*/
	uint64_t secFromEpoch = UnixEpoch.Seconds;
	AppData.Buffer[i++] = (uint8_t) ((secFromEpoch >> 24) & 0xFF);
	AppData.Buffer[i++] = (uint8_t) ((secFromEpoch >> 16) & 0xFF);
	AppData.Buffer[i++] = (uint8_t) ((secFromEpoch >> 8) & 0xFF);
	AppData.Buffer[i++] = (uint8_t) (secFromEpoch & 0xFF);
	APP_LOG(TS_ON, VLEVEL_M, "Time request succsess 1\r\n");
	AppData.BufferSize = i;
	if (LORAMAC_HANDLER_SUCCESS
			== LmHandlerSend(&AppData, LORAMAC_HANDLER_CONFIRMED_MSG, true)) {
		APP_LOG(TS_OFF, VLEVEL_H, "SEND REQUEST\n");
	} else if (nextTxIn > 0) {
		APP_LOG(TS_OFF, VLEVEL_L, "Next Tx in  : ~%d second(s)\n",
				(nextTxIn / 1000));
	}
}

void sendTiltTamperAlert(void) {
	uint8_t i = 0;
	AppData.Port = LORAWAN_USER_APP_PORT;

	// Payload for Tamper Alert
	AppData.Buffer[i++] = 0x0F; // Tamper Alert Identifier
	AppData.Buffer[i++] = 0x01; // Tamper detected

	// time stamp payload
	SysTime_t UnixEpoch = SysTimeGet();
	UnixEpoch.Seconds -= 18; /*removing leap seconds*/
	UnixEpoch.Seconds += 19800; /*adding 5:30 hours*/
	uint64_t secFromEpoch = UnixEpoch.Seconds;
	AppData.Buffer[i++] = (uint8_t) ((secFromEpoch >> 24) & 0xFF);
	AppData.Buffer[i++] = (uint8_t) ((secFromEpoch >> 16) & 0xFF);
	AppData.Buffer[i++] = (uint8_t) ((secFromEpoch >> 8) & 0xFF);
	AppData.Buffer[i++] = (uint8_t) (secFromEpoch & 0xFF);

	AppData.BufferSize = i;

	// Ensure message is confirmed for security
		LmHandlerParams.IsTxConfirmed = LORAMAC_HANDLER_CONFIRMED_MSG;
		// Set this to true so that OnTxData will retry if the message fails
		is_critical_event_pending = true;
		if (LORAMAC_HANDLER_SUCCESS== LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, true)) {
			APP_LOG(TS_ON, VLEVEL_L, "TAMPER ALERT SEND REQUEST\n");
		} else {
			APP_LOG(TS_ON, VLEVEL_L, "TAMPER ALERT SEND FAILED\n");
	}
}

void sendResponceData(uint8_t state, uint8_t slaveId, uint8_t functionCode,
		uint16_t registerAddress, uint16_t registerValue) {
	uint8_t i = 0;
	//uint16_t battery = getBattVoltage();
	// APP_LOG(TS_ON, VLEVEL_M, "BAT Volt:%d\n", battery);
	UTIL_TIMER_Time_t nextTxIn = 0;

	// new tamper pyaload

	AppData.Port = LORAWAN_USER_APP_PORT;
	// mode 3
	AppData.Buffer[i++] = 3;

	// write status
	AppData.Buffer[i++] = state;

	// slave id
	AppData.Buffer[i++] = slaveId;

	// function code
	AppData.Buffer[i++] = functionCode;

	//register
	AppData.Buffer[i++] = (uint8_t) ((registerAddress >> 8) & 0xFF);
	AppData.Buffer[i++] = (uint8_t) (registerAddress & 0xFF);

	//
	//register value
	AppData.Buffer[i++] = (uint8_t) ((registerValue >> 8) & 0xFF);
	AppData.Buffer[i++] = (uint8_t) (registerValue & 0xFF);

	// time stamp payload
	SysTime_t UnixEpoch = SysTimeGet();
	UnixEpoch.Seconds -= 18; /*removing leap seconds*/
	UnixEpoch.Seconds += 19800; /*adding 5:30 hours*/
	uint64_t secFromEpoch = UnixEpoch.Seconds;
	AppData.Buffer[i++] = (uint8_t) ((secFromEpoch >> 24) & 0xFF);
	AppData.Buffer[i++] = (uint8_t) ((secFromEpoch >> 16) & 0xFF);
	AppData.Buffer[i++] = (uint8_t) ((secFromEpoch >> 8) & 0xFF);
	AppData.Buffer[i++] = (uint8_t) (secFromEpoch & 0xFF);

	AppData.BufferSize = i;
	if (LORAMAC_HANDLER_SUCCESS
			== LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, false)) {
		APP_LOG(TS_ON, VLEVEL_L, "SEND REQUEST\n");
	} else if (nextTxIn > 0) {
		APP_LOG(TS_ON, VLEVEL_L, "Next Tx in  : ~%d second(s)\n",
				(nextTxIn / 1000));
	}
}

uint16_t getBattVoltage(void) {
	HAL_Delay(5);
	uint16_t ADCxConvertedValues = 0;
	ADC_ChannelConfTypeDef sConfig = { 0 };

	MX_ADC_Init();

	/* Start Calibration */
	if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK) {
		Error_Handler();
	}

	/* Configure Regular Channel */

	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_ADC_Start(&hadc) != HAL_OK) {
		/* Start Error */
		Error_Handler();
	}
	/** Wait for end of conversion */
	HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);

	/** Wait for end of conversion */
	HAL_ADC_Stop(&hadc); /* it calls also ADC_Disable() */

	ADCxConvertedValues = HAL_ADC_GetValue(&hadc);

	HAL_ADC_DeInit(&hadc);
	HAL_Delay(1);

	return ADCxConvertedValues;
}

void updateActiveRegion(int32_t region) {
	switch (region) {
	case 0:
		LmHandlerParams.ActiveRegion = LORAMAC_REGION_AS923;
		break;
	case 1:
		LmHandlerParams.ActiveRegion = LORAMAC_REGION_AU915;
		break;
	case 2:
		LmHandlerParams.ActiveRegion = LORAMAC_REGION_EU868;	//
		break;
	case 3:
		LmHandlerParams.ActiveRegion = LORAMAC_REGION_IN865;	//
		break;
	case 4:
		LmHandlerParams.ActiveRegion = LORAMAC_REGION_US915;	//
		break;
		// Add more cases for other regions as needed
	default:
		// Invalid region, do nothing or handle the error
		EEPROM_Write_uint32(REGION_ADDRESS, 3); // is indian frequency
		LmHandlerParams.ActiveRegion = LORAMAC_REGION_IN865;
		break;
	}
}

void lorawanClass(char class) {
	switch (class) {
	case 'A':
		LmHandlerParams.DefaultClass = CLASS_A;
		break;
	case 'B':
		LmHandlerParams.DefaultClass = CLASS_B;
		break;
	case 'C':
		LmHandlerParams.DefaultClass = CLASS_C;
		break;
	default:
		LmHandlerParams.DefaultClass = CLASS_A;
		break;
	}
}

void dataRate(uint8_t data_rate) {
	switch (data_rate) {
	case 0:
		LmHandlerParams.TxDatarate = DR_0;
		break;
	case 1:
		LmHandlerParams.TxDatarate = DR_1;
		break;
	case 2:
		LmHandlerParams.TxDatarate = DR_2;
		break;
	case 3:
		LmHandlerParams.TxDatarate = DR_3;
		break;
	case 4:
		LmHandlerParams.TxDatarate = DR_4;
		break;
	case 5:
		LmHandlerParams.TxDatarate = DR_5;
		break;
	case 6:
		LmHandlerParams.TxDatarate = DR_6;
		break;
	case 7:
		LmHandlerParams.TxDatarate = DR_7;
		break;
	case 8:
		LmHandlerParams.TxDatarate = DR_8;
		break;
	case 9:
		LmHandlerParams.TxDatarate = DR_9;
		break;
	case 10:
		LmHandlerParams.TxDatarate = DR_10;
		break;
	case 11:
		LmHandlerParams.TxDatarate = DR_11;
		break;
	case 12:
		LmHandlerParams.TxDatarate = DR_12;
		break;
	case 13:
		LmHandlerParams.TxDatarate = DR_13;
		break;
	case 14:
		LmHandlerParams.TxDatarate = DR_14;
		break;
	default:
		LmHandlerParams.TxDatarate = DR_0;
		break;

	}
}

//
HAL_StatusTypeDef initilzeLorawanVariables(void) {

	int32_t BAND = EEPROM_Read_uint32(REGION_ADDRESS);
	updateActiveRegion(BAND);
	int32_t Interval = EEPROM_Read_uint32(4);
	if (Interval < 300 || Interval > 86400) {
		TX_DUTYCYCLE = 10;
		EEPROM_Write_uint32(4, 10);
		APP_LOG(TS_OFF, VLEVEL_M,
				"New Chip Detected !! Saving Default Data...\n");
		// saveDefaultModbusData();
	} else {
		TX_DUTYCYCLE = EEPROM_Read_uint32(4);

	}

	UPLINK_TIMER = EEPROM_Read_uint32(UPLINK_TIMER_ADDRESS);
	if (UPLINK_TIMER < 300 || UPLINK_TIMER > 86400) {
		UPLINK_TIMER = 43200; //Set default to 12 hours
		EEPROM_Write_uint32(UPLINK_TIMER_ADDRESS, UPLINK_TIMER);
	}
//	APP_LOG(TS_OFF, VLEVEL_M, "###### CONFIG ######\n");
//	APP_LOG(TS_OFF, VLEVEL_M, "###### TXTIME: %d\n", TX_DUTYCYCLE);

	char class = EEPROM_Read(12);
	if (class == 'A' || class == 'B' || class == 'C') {
		lorawanClass(class);
	} else {
		class = 'A';
		EEPROM_Write(12, class);
		lorawanClass(class);
	}
	APP_LOG(TS_OFF, VLEVEL_M, "###### CLASS: %c\n", class);

	ActivationType = ACTIVATION_TYPE_OTAA;
	APP_LOG(TS_OFF, VLEVEL_M, "###### JOIN: OTAA\n", class);

	//
	int8_t ADR = EEPROM_Read(ADR_ADDRESS);
	if (ADR == 0) {
		LmHandlerParams.AdrEnable = LORAMAC_HANDLER_ADR_OFF;
		APP_LOG(TS_OFF, VLEVEL_M, "###### ADR: ADR_OFF\n");
	} else if (ADR == 1) {
		LmHandlerParams.AdrEnable = LORAMAC_HANDLER_ADR_ON;
		APP_LOG(TS_OFF, VLEVEL_M, "###### ADR: ADR_ON\n");
	} else {
		EEPROM_Write(ADR_ADDRESS, 1);
		LmHandlerParams.AdrEnable = LORAMAC_HANDLER_ADR_ON;
		APP_LOG(TS_OFF, VLEVEL_M, "###### ADR: ADR_ON\n");
	}

	//
	int8_t DataRate = EEPROM_Read(DATA_RATE_ADDRESS);	 	//Data Rate
	if (DataRate >= 0 && DataRate <= 14) {
		dataRate(DataRate);
		APP_LOG(TS_OFF, VLEVEL_M, "###### DR: DR_%d\n", DataRate);
	} else {
		DataRate = 0;
		EEPROM_Write(DATA_RATE_ADDRESS, DataRate);
		dataRate(DataRate);
		APP_LOG(TS_OFF, VLEVEL_M, "###### DR: DR_%d\n", DataRate);
	}

	// message type
	int8_t msgType = EEPROM_Read(MSG_TYPE_ADDRESS);
	if (msgType == 0) {
		messageType = LORAMAC_HANDLER_UNCONFIRMED_MSG;
		APP_LOG(TS_OFF, VLEVEL_M, "###### MSTYPE: UNCONFIRMED_MSG\n", class);
	} else if (msgType == 1) {
		messageType = LORAMAC_HANDLER_CONFIRMED_MSG;
		APP_LOG(TS_OFF, VLEVEL_M, "###### MSTYPE: CONFIRMED_MSG\n", class);
	} else {
		EEPROM_Write(MSG_TYPE_ADDRESS, 1);
		messageType = LORAMAC_HANDLER_CONFIRMED_MSG;
		APP_LOG(TS_OFF, VLEVEL_M, "###### MSTYPE: CONFIRMED_MSG\n", class);
	}

	return HAL_OK;
}

void lis2mdl_parking_sensor() {
	char log_buffer[64];
	uint8_t i = 0;
	SysTime_t currentTime = SysTimeGet();
	uint32_t msSinceLastUp = SysTimeToMs(currentTime)
			- SysTimeToMs(lastUplinkTime);
	bool First_Uplink = false;

	AppData.Buffer[i++] = 0x10;  // Device identity

	// ---  read sensor ---Wake I2C bus and re-init sensor from power-down mode
	MX_I2C1_Init();
	HAL_Delay(5);
	LIS2MDL_ReInit(&hi2c1);
	int16_t x,y,z;

	// first uplink First-time calibration: store current reading as "empty spot" baseline
	if (!joinStatus) {
		x = LIS2MDL_ReadAxis(REG_OUTX_L);
		y = LIS2MDL_ReadAxis(REG_OUTY_L);
		z = LIS2MDL_ReadAxis(REG_OUTZ_L);
        x_initial_reading = x;
        y_initial_reading = y;
        z_initial_reading = z;
		joinStatus = true;
		First_Uplink = true;
		AppData.Buffer[i++] = 0x01;  // Event identifier
		AppData.Buffer[i++] = 0x00;    // Status: no parking (calibration)
		HAL_Delay(20);   // Wait for sensor to make new fresh reading
		if (LmHandlerDeviceTimeReq() == LORAMAC_HANDLER_SUCCESS) { // Time Synchronization
			APP_LOG(TS_ON, VLEVEL_M, "Time request succsess\n");
		}
	}
	// Read current magnetic field values
	x = LIS2MDL_ReadAxis(REG_OUTX_L);
	y = LIS2MDL_ReadAxis(REG_OUTY_L);
	z = LIS2MDL_ReadAxis(REG_OUTZ_L);



	snprintf(log_buffer, sizeof(log_buffer), "Raw Mag: X=%d Y=%d Z=%d\r\n", x,
			y, z);
	APP_LOG(TS_OFF, VLEVEL_M, "%s", log_buffer);

	snprintf(log_buffer, sizeof(log_buffer), "Difference: X=%d Y=%d Z=%d uT\n",
			x - x_initial_reading, y - y_initial_reading,
			z - z_initial_reading);

	APP_LOG(TS_OFF, VLEVEL_M, "%s", log_buffer);

	// Parking detection: car present if 2+ axes deviate > 25uT from baseline
	int x_diff = abs(x - x_initial_reading);
	int y_diff = abs(y - y_initial_reading);
	int z_diff = abs(z - z_initial_reading);

	int condition_count = 0;
	if (x_diff > 25)
		condition_count++;
	if (y_diff > 25)
		condition_count++;
	if (z_diff > 25)
		condition_count++;

	bool parking_now = (condition_count >= 2);

	// Detect if parking state is transitioning or stable
	bool state_is_changing = (parking_now != last_uplink_parking_state);

	// Track consecutive same-state readings for 3x confirmation
	if (parking_now) {
		Parking_detection_counter++;
		NoParking_detection_counter = 0;
	} else {
		NoParking_detection_counter++;
		Parking_detection_counter = 0;
	}

	// Dynamic polling: fast 3s during state change, normal 10s when stable
	if (state_is_changing) {
		TX_DUTYCYCLE = 3; // Fast polling: 3s+3s+3s to confirm state change
	} else {
		TX_DUTYCYCLE = 10; // Normal monitoring interval
	}

	APP_LOG(TS_OFF, VLEVEL_M,
			"Parking: %s, parking_count=%d, noparking_count=%d\n",
			(parking_now ? "Detected" : "Not Detected"),
			Parking_detection_counter, NoParking_detection_counter);

	// --- Uplink decision: retry > 3x state change > heartbeat timeout ---
	bool sendUplink = false;

	// Priority 1: Active retry cycle — force re-send without counter check
	if (is_critical_event_pending && event_retry_counter > 0) {
		sendUplink = true;
		APP_LOG(TS_OFF, VLEVEL_M,
				">>> RETRY TRIGGERED: Forcing Uplink (Attempt %d)\n",
				event_retry_counter);

		// Use the last target state for the retry payload
		AppData.Buffer[i++] = 0x00;  // Event: state change
		AppData.Buffer[i++] = parking_now;

   // Priority 2: 3x consecutive PARKING confirmed (was empty before)
	} else if (Parking_detection_counter >= 3 && !last_uplink_parking_state) {
		AppData.Buffer[i++] = 0x00;   // Event identifier
		AppData.Buffer[i++] = parking_now;   // Parking status identifier
		is_critical_event_pending = true;
		event_retry_counter = 0;

		APP_LOG(TS_OFF, VLEVEL_M,
				"3x consecutive PARKING -> immediate uplink\n");
		sendUplink = true;

   // Priority 3: 3x consecutive NO PARKING confirmed (was occupied before)
	} else if (NoParking_detection_counter >= 3 && last_uplink_parking_state) {
		AppData.Buffer[i++] = 0x00;  // Event identifier
		AppData.Buffer[i++] = parking_now;   // No Parking status identifier
		is_critical_event_pending = true;
		event_retry_counter = 0;
		APP_LOG(TS_OFF, VLEVEL_M,
				"3x consecutive NO PARKING -> immediate uplink\n");
		sendUplink = true;

    // Priority 4: Heartbeat timer expired (default 12h)
	} else if (lastUplinkTime.Seconds > 0
			&& msSinceLastUp >= (uint32_t) UPLINK_TIMER * 1000) {

		sendUplink = true;
		is_critical_event_pending = false;
		AppData.Buffer[i++] = 0x01;// Event: heartbeat
		AppData.Buffer[i++] = parking_now ? 0x01 : 0x00;
	}

	// Append battery + timestamp to every payload
	AppData.Buffer[i++] = Battery_GetPercent();
	SysTime_t ue = SysTimeGet();
	ue.Seconds -= 18;
	uint64_t sec = ue.Seconds;
	AppData.Buffer[i++] = (sec >> 24) & 0xFF;
	AppData.Buffer[i++] = (sec >> 16) & 0xFF;
	AppData.Buffer[i++] = (sec >> 8) & 0xFF;
	AppData.Buffer[i++] = (sec) & 0xFF;

	// Append battery percentage identifier and value to the payload

	AppData.Port = LORAWAN_USER_APP_PORT;
	AppData.BufferSize = i;

	// --- Transmit if any uplink condition was met ---

	if (sendUplink || First_Uplink) {
		// Read message type from EEPROM (confirmed or unconfirmed)
		uint8_t messageType = EEPROM_Read(MSG_TYPE_ADDRESS);
		LmHandlerParams.IsTxConfirmed =
				(messageType == 1) ?
						LORAMAC_HANDLER_CONFIRMED_MSG :
						LORAMAC_HANDLER_UNCONFIRMED_MSG;

		float remaining_mAh = Battery_GetRemaining_mAh();
		int percent = Battery_GetPercent();
		int rem_int = (int) remaining_mAh;
		int rem_dec = (int) ((remaining_mAh - rem_int) * 100);
		APP_LOG(TS_OFF, VLEVEL_M, "Battery -> Remaining: %d.%02d mAh | %d %%\n",
				rem_int, rem_dec, percent);

		Battery_StartTx();// Track TX power consumption
		if (LORAMAC_HANDLER_SUCCESS
				== LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed,
				true)) {
			APP_LOG(TS_OFF, VLEVEL_L, "Uplink sent\n");
			lastUplinkTime = currentTime;
			last_uplink_parking_state = parking_now;

			// After successful uplink, resume normal 10s monitoring
			TX_DUTYCYCLE = 10;
		} else {
			APP_LOG(TS_OFF, VLEVEL_L, "Uplink FAILED\n");
		}

		// Reset both counters
		Parking_detection_counter = 0;
		NoParking_detection_counter = 0;
	}
	// Power down sensor and release I2C to save battery between readings
	HAL_Delay(1);
	LIS2MDL_PowerDown();
	HAL_Delay(1);
	LIS2MDL_DeInit(&hi2c1);
	HAL_I2C_DeInit(&hi2c1); //This fully shuts down the I2C1 hardware
	First_Uplink = false;
}

static void OnRxData(LmHandlerAppData_t *appData, LmHandlerRxParams_t *params) {
	/* USER CODE BEGIN OnRxData_1 */
	if ((appData != NULL) || (params != NULL)) {
		APP_LOG(TS_OFF, VLEVEL_H, "APP PORT:%d\n", appData->Port);
		APP_LOG(TS_OFF, VLEVEL_H, "DATA[0]:%d\n", appData->Buffer[0]);

		static const char *slotStrings[] = { "1", "2", "C", "C Multicast",
				"B Ping-Slot", "B Multicast Ping-Slot" };

		APP_LOG(TS_OFF, VLEVEL_H,
				"\n###### ========== MCPS-Indication ==========\n");
		APP_LOG(TS_OFF, VLEVEL_H,
				"###### D/L FRAME:%04d | SLOT:%s | PORT:%d | DR:%d | RSSI:%d | SNR:%d\n",
				params->DownlinkCounter, slotStrings[params->RxSlot],
				appData->Port, params->Datarate, params->Rssi, params->Snr);
		if (uplinkStatus) {
			APP_LOG(TS_OFF, VLEVEL_M, "UPSUCCESS=%04d,%d,%d,%d,%d,%d\n",
					params->DownlinkCounter, uplinkStatus, appData->Port,
					params->Datarate, params->Rssi, params->Snr);
		}
		uplinkStatus = false;
		if (TimerState) {
			UTIL_TIMER_Stop(&TxTimer);
			UTIL_TIMER_Start(&TriggerTimer);
			TimerState = false;
			APP_LOG(TS_OFF, VLEVEL_M, "Trigger Timer Start\n");
		}
		LmHandlerParams.IsTxConfirmed = LORAMAC_HANDLER_UNCONFIRMED_MSG;
		switch (appData->Port) {
		case LORAWAN_SWITCH_CLASS_PORT:
			/*this port switches the class*/
			if (appData->BufferSize == 1) {
				switch (appData->Buffer[0]) {
				case 0: {
					LmHandlerRequestClass(CLASS_A);
					break;
				}
				case 1: {
					LmHandlerRequestClass(CLASS_B);
					break;
				}
				case 2: {
					LmHandlerRequestClass(CLASS_C);
					break;
				}
				default:
					break;
				}
			}
			break;
		case LORAWAN_TXINT_PORT:
			if (appData->BufferSize == 2) {
				uint16_t recivedTx = ((uint16_t) appData->Buffer[0] << 8)
						| appData->Buffer[1];
				if (recivedTx >= 10) {
					APP_LOG(TS_OFF, VLEVEL_M, "Tx Time recived : %d sec\n",
							recivedTx);
					TX_DUTYCYCLE = recivedTx;
					EEPROM_Write_uint32(4, TX_DUTYCYCLE);
					APP_LOG(TS_OFF, VLEVEL_M, "Tx Time : %d Sec\n",
							TX_DUTYCYCLE);
					UTIL_TIMER_SetPeriod(&TxTimer, (TX_DUTYCYCLE * 1000));
					UTIL_TIMER_Start(&TxTimer);
					sendTamperData();
				}
			}
			break;
		case LORAWAN_BOARD_RESET_PORT:
			MX_I2C1_Init();
			HAL_Delay(10);  // Wait for sensor to stabilize
			LIS2MDL_Init(&hi2c1);
			HAL_Delay(100);  // Wait for sensor to stabilize
			HAL_NVIC_SystemReset();
			break;

		case UPLINK_TIMER_PORT:
			if (appData->BufferSize == 2) {
				uint16_t recivedTx = ((uint16_t) appData->Buffer[0] << 8)
						| appData->Buffer[1];
				if (recivedTx >= 300) {
					APP_LOG(TS_OFF, VLEVEL_M, "Tx Time recived : %d sec\n",
							recivedTx);
					UPLINK_TIMER = recivedTx;
					EEPROM_Write_uint32(UPLINK_TIMER_ADDRESS, UPLINK_TIMER);
					APP_LOG(TS_OFF, VLEVEL_M, "Tx Time : %d Sec\n",
							UPLINK_TIMER);
					UTIL_TIMER_SetPeriod(&TxTimer, (UPLINK_TIMER * 1000));
					UTIL_TIMER_Start(&TxTimer);
					sendTamperData();
				}
			}
			break;
		case ADR_PORT:
			if (appData->BufferSize == 2) {
				uint8_t recivedADR = appData->Buffer[0];
				uint8_t DataRate = appData->Buffer[1];
				if ((recivedADR == 0 || recivedADR == 1)
						&& (DataRate >= 0 && DataRate <= 5)) {
					APP_LOG(TS_OFF, VLEVEL_M, "ADR : %d\n", recivedADR);
					EEPROM_Write(ADR_ADDRESS, recivedADR);
					EEPROM_Write(DATA_RATE_ADDRESS, DataRate);
					APP_LOG(TS_OFF, VLEVEL_M, "ADR : %d, DATA_RATE : %d\n",
							recivedADR, DataRate);
					HAL_Delay(1000);
					HAL_NVIC_SystemReset();
				}
			}
		case MSG_TYPE_PORT:
			if (appData->BufferSize == 1) {
				uint8_t msgType = appData->Buffer[0];
				if (msgType == 0 || msgType == 1) {
					APP_LOG(TS_OFF, VLEVEL_M, "msgType : %d\n", msgType);
					EEPROM_Write(MSG_TYPE_ADDRESS, msgType);
				}
			}
			break;
		default:

			break;
		}
		HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
	}
	//LmHandlerParams.IsTxConfirmed = LORAMAC_HANDLER_UNCONFIRMED_MSG;
	/* USER CODE END OnRxData_1 */
}

static void SendTxData(void) {
	/* USER CODE BEGIN SendTxData_1 */
	// 1. Check if we are in Permanent Sleep mode (Shutdown)
		if (isPermanentSleep) {
			APP_LOG(TS_OFF, VLEVEL_M, "SHUTDOWN: Device is in Permanent Sleep. No action taken.\n");
			return;
		}
		// 2. Handle Tamper Alert Burst (3 messages with 15s gaps)
		if (tiltTamperDetected || (tamperBurstCounter > 0 && tamperBurstCounter < 3)) {

			if (tiltTamperDetected) {
				APP_LOG(TS_ON, VLEVEL_M, "Tamper Triggered! Starting 3-message security burst.\n");
				tiltTamperDetected = false;
				tamperBurstCounter = 0; // Reset counter for new burst sequence
			}
			tamperBurstCounter++;
			APP_LOG(TS_ON, VLEVEL_M, "TAMPER BURST: Sending alert %d of 3...\n", tamperBurstCounter);

			sendTiltTamperAlert();

			if (tamperBurstCounter < 3) {
				// Schedule the next message in exactly 15 seconds
				UTIL_TIMER_Stop(&TxTimer);
				UTIL_TIMER_SetPeriod(&TxTimer, 15000);
				UTIL_TIMER_Start(&TxTimer);
			} else {
				// Burst complete: Shut down the device permanently
				APP_LOG(TS_ON, VLEVEL_M, "!!! SECURITY SHUTDOWN: Burst Complete. Entering Permanent Sleep !!!\n");
				isPermanentSleep = true;

				// Stop all timers to ensure the device stays in STOP2 forever
				UTIL_TIMER_Stop(&TxTimer);
				UTIL_TIMER_Stop(&TriggerTimer);
				UTIL_TIMER_Stop(&JoinBackoffTimer);
			}
			return; // Exit to prevent normal sensor logic from running
		}
		 else if (waiting_for_first_ack) {
			// Priority 3: Re-send SAME payload (no rebuild to preserve server format)
			APP_LOG(TS_ON, VLEVEL_M, "Re-sending first uplink...\n");
			Battery_StartTx();
			if (LORAMAC_HANDLER_SUCCESS
					== LmHandlerSend(&AppData, LmHandlerParams.IsTxConfirmed, true)) {
				APP_LOG(TS_OFF, VLEVEL_L, "Uplink re-sent\n");
			} else {
				APP_LOG(TS_OFF, VLEVEL_L, "Uplink re-send FAILED\n");
				    // Retry again in 5s
				    UTIL_TIMER_Stop(&TxTimer);
				    UTIL_TIMER_SetPeriod(&TxTimer, 5000);
				    UTIL_TIMER_Start(&TxTimer);
		}
		} else if (sendJoiningMsg) {
			// Priority 4: First-ever uplink — build payload, set ACK gate
			APP_LOG(TS_ON, VLEVEL_M, "Sending Time Req Msg!\n");
			sendJoiningMsg = false;
			lis2mdl_parking_sensor();
			waiting_for_first_ack = true;
		} else {
			// Priority 5: Normal sensor polling cycle
		      lis2mdl_parking_sensor();

		   // Only restart timer if OnTxData isn't managing retries
		      if (!is_retry_cycle) {
			  UTIL_TIMER_Stop(&TxTimer);
			  UTIL_TIMER_SetPeriod(&TxTimer, (TX_DUTYCYCLE * 1000));
			  UTIL_TIMER_Start(&TxTimer);
		}
		is_retry_cycle = false; // Clear for next normal cycle
	}
}
static void OnTxTimerEvent(void *context) {
	/* USER CODE BEGIN OnTxTimerEvent_1 */

	/* USER CODE END OnTxTimerEvent_1 */
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent),
			CFG_SEQ_Prio_0);

	/* USER CODE BEGIN OnTxTimerEvent_2 */

	/* USER CODE END OnTxTimerEvent_2 */
}

static void OnTxData(LmHandlerTxParams_t *params) {
	Battery_EndTx();// Stop TX power consumption tracking
	if ((params != NULL) && (params->IsMcpsConfirm != 0)) {

		// === FIRST UPLINK ACK GATE ===
		if (waiting_for_first_ack) {
		    if (params->AckReceived != 0) {
		    	// ACK received — unlock sensor timer, begin normal monitoring
		        waiting_for_first_ack = false;
		        APP_LOG(TS_OFF, VLEVEL_M, ">>> FIRST UPLINK: ACK Received. Starting sensor timer.\n");
		        UTIL_TIMER_Stop(&TxTimer);
		        UTIL_TIMER_SetPeriod(&TxTimer, (10 * 1000));
		        UTIL_TIMER_Start(&TxTimer);
		    } else {
		    	// No ACK — retry same payload in 5s, sensor timer stays blocked
		        APP_LOG(TS_OFF, VLEVEL_M, ">>> FIRST UPLINK: No ACK. Retrying in 5s...\n");
		        //sendJoiningMsg = true;
		        UTIL_TIMER_Stop(&TxTimer);
		        UTIL_TIMER_SetPeriod(&TxTimer, 5000);
		        UTIL_TIMER_Start(&TxTimer);
		    }
		    return; // Skip normal retry logic during first-uplink phase
		   }

		// === NORMAL MODE: ACK/NACK for parking events ===
		        if (params->AckReceived != 0) {
		        	// SUCCESS: Event delivered, reset all retry state
		            is_critical_event_pending = false;
		            event_retry_counter = 0;
		            is_retry_cycle = false;
		            APP_LOG(TS_OFF, VLEVEL_M, ">>> SUCCESS: ACK Received.\n");

		            // ONLY restart normal timer if we are NOT in a burst or shutdown
		            if (!isPermanentSleep && tamperBurstCounter == 0) {
		            UTIL_TIMER_Stop(&TxTimer);
		            UTIL_TIMER_SetPeriod(&TxTimer, (10 * 1000));
		            UTIL_TIMER_Start(&TxTimer);
		           }
		        }
		        else {
		        	// NACK: Retry only if this was a critical parking state change
			       if (is_critical_event_pending == true) {
				      if (event_retry_counter < MAX_EVENT_RETRIES) {
					      event_retry_counter++;
					// Random 2-6s jitter to avoid collision with other devices
					uint32_t random_jitter = (Radio.Random() % 4000);
					uint32_t next_retry_delay = 2000 + random_jitter;
					is_retry_cycle = true;  // Tell SendTxData: don't touch timer
					UTIL_TIMER_Stop(&TxTimer);
					UTIL_TIMER_SetPeriod(&TxTimer, next_retry_delay);
					UTIL_TIMER_Start(&TxTimer);
				} else {
					// Max 6 retries exhausted — give up, resume normal polling
					APP_LOG(TS_OFF, VLEVEL_M,
							">>> CRITICAL: Max retries reached.\n");
					is_critical_event_pending = false;
					event_retry_counter = 0;
					is_retry_cycle = false;
					if (tamperBurstCounter == 0) {
					UTIL_TIMER_Stop(&TxTimer);
					UTIL_TIMER_SetPeriod(&TxTimer, (10 * 1000));
					UTIL_TIMER_Start(&TxTimer);
				}
			}
		 }
	  }
   }
}
static void OnJoinRequest(LmHandlerJoinParams_t *joinParams) {
	/* USER CODE BEGIN OnJoinRequest_1 */
	if (joinParams != NULL) {
		if (joinParams->Status == LORAMAC_HANDLER_SUCCESS) {
			// Reset join retry counter on success
			join_retry_counter = 0;

			// Initialize I2C and LIS2MDL sensor for first calibration read
			MX_I2C1_Init();
			HAL_Delay(100);  // Wait for sensor to stabilize
			LIS2MDL_Init(&hi2c1);

			APP_LOG(TS_OFF, VLEVEL_M, "\n###### = JOINED SUCCESS = \n");

			// Wait 5s for gateway to recover from join-accept TX before first uplink
			    UTIL_TIMER_Stop(&TxTimer);
			    UTIL_TIMER_SetPeriod(&TxTimer, 5000);
			    UTIL_TIMER_Start(&TxTimer);

		} else {
			// Join failed — increment counter and schedule retry
			join_retry_counter++;
			APP_LOG(TS_OFF, VLEVEL_M,
					"\n###### = JOIN FAILED (Attempt %d/12)\n",
					join_retry_counter);
			if (join_retry_counter < MAX_JOIN_ATTEMPTS) {
				// Random 2-8s delay before retry to avoid gateway collision
				uint32_t join_delay = 2000 + (Radio.Random() % 6000);
				APP_LOG(TS_OFF, VLEVEL_M, "Join retry in %u ms\n", join_delay);
				UTIL_TIMER_Stop(&JoinBackoffTimer);
				UTIL_TIMER_SetPeriod(&JoinBackoffTimer, join_delay);
				UTIL_TIMER_Start(&JoinBackoffTimer);
			} else {
				// 12 attempts exhausted — enter 24-hour deep sleep
				APP_LOG(TS_OFF, VLEVEL_M, "### MAX RETRIES REACHED....\n");
				UTIL_TIMER_Stop(&JoinBackoffTimer);
				UTIL_TIMER_SetPeriod(&JoinBackoffTimer, JOIN_BACKOFF_TIME);
				UTIL_TIMER_Start(&JoinBackoffTimer);
			}
		}
	}
	/* USER CODE END OnJoinRequest_1 */
}

static void OnBeaconStatusChange(LmHandlerBeaconParams_t *params) {
	/* USER CODE BEGIN OnBeaconStatusChange_1 */
	/* USER CODE END OnBeaconStatusChange_1 */
}

static void OnSysTimeUpdate(void) {
	/* USER CODE BEGIN OnSysTimeUpdate_1 */
	Battery_Init();
	APP_LOG(TS_OFF, VLEVEL_M, ">>> RTC UPDATED.\n");

	/* USER CODE END OnSysTimeUpdate_1 */
}


static void OnClassChange(DeviceClass_t deviceClass) {
	/* USER CODE BEGIN OnClassChange_1 */
	/* USER CODE END OnClassChange_1 */
}

static void OnMacProcessNotify(void) {
	/* USER CODE BEGIN OnMacProcessNotify_1 */

	/* USER CODE END OnMacProcessNotify_1 */
	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LmHandlerProcess), CFG_SEQ_Prio_0);

	/* USER CODE BEGIN OnMacProcessNotify_2 */

	/* USER CODE END OnMacProcessNotify_2 */
}

static void OnTxPeriodicityChanged(uint32_t periodicity) {
	/* USER CODE BEGIN OnTxPeriodicityChanged_1 */

	/* USER CODE END OnTxPeriodicityChanged_1 */
	TxPeriodicity = periodicity;

	if (TxPeriodicity == 0) {
		/* Revert to application default periodicity */
		TxPeriodicity = APP_TX_DUTYCYCLE;
	}

	/* Update timer periodicity */
	UTIL_TIMER_Stop(&TxTimer);
	UTIL_TIMER_SetPeriod(&TxTimer, TxPeriodicity);
	UTIL_TIMER_Start(&TxTimer);
	/* USER CODE BEGIN OnTxPeriodicityChanged_2 */

	/* USER CODE END OnTxPeriodicityChanged_2 */
}

static void OnTxFrameCtrlChanged(LmHandlerMsgTypes_t isTxConfirmed) {
	/* USER CODE BEGIN OnTxFrameCtrlChanged_1 */

	/* USER CODE END OnTxFrameCtrlChanged_1 */
	LmHandlerParams.IsTxConfirmed = isTxConfirmed;
	/* USER CODE BEGIN OnTxFrameCtrlChanged_2 */

	/* USER CODE END OnTxFrameCtrlChanged_2 */
}

static void OnPingSlotPeriodicityChanged(uint8_t pingSlotPeriodicity) {
	/* USER CODE BEGIN OnPingSlotPeriodicityChanged_1 */

	/* USER CODE END OnPingSlotPeriodicityChanged_1 */
	LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
	/* USER CODE BEGIN OnPingSlotPeriodicityChanged_2 */

	/* USER CODE END OnPingSlotPeriodicityChanged_2 */
}

static void OnSystemReset(void) {
	/* USER CODE BEGIN OnSystemReset_1 */

	/* USER CODE END OnSystemReset_1 */
	if ((LORAMAC_HANDLER_SUCCESS == LmHandlerHalt())
			&& (LmHandlerJoinStatus() == LORAMAC_HANDLER_SET)) {
		NVIC_SystemReset();
	}
	/* USER CODE BEGIN OnSystemReset_Last */

	/* USER CODE END OnSystemReset_Last */
}

static void StopJoin(void) {
	/* USER CODE BEGIN StopJoin_1 */

	/* USER CODE END StopJoin_1 */

	UTIL_TIMER_Stop(&TxTimer);

	if (LORAMAC_HANDLER_SUCCESS != LmHandlerStop()) {
		APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stop on going ...\r\n");
	} else {
		APP_LOG(TS_OFF, VLEVEL_M, "LmHandler Stopped\r\n");
		if (LORAWAN_DEFAULT_ACTIVATION_TYPE == ACTIVATION_TYPE_ABP) {
			ActivationType = ACTIVATION_TYPE_OTAA;
			APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to OTAA mode\r\n");
		} else {
			ActivationType = ACTIVATION_TYPE_ABP;
			APP_LOG(TS_OFF, VLEVEL_M, "LmHandler switch to ABP mode\r\n");
		}
		LmHandlerConfigure(&LmHandlerParams);
		LmHandlerJoin(ActivationType, true);
		UTIL_TIMER_Start(&TxTimer);
	}
	UTIL_TIMER_Start(&StopJoinTimer);
	/* USER CODE BEGIN StopJoin_Last */

	/* USER CODE END StopJoin_Last */
}

static void OnStopJoinTimerEvent(void *context) {
	/* USER CODE BEGIN OnStopJoinTimerEvent_1 */

	/* USER CODE END OnStopJoinTimerEvent_1 */
	if (ActivationType == LORAWAN_DEFAULT_ACTIVATION_TYPE) {
		UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaStopJoinEvent), CFG_SEQ_Prio_0);
	}
	/* USER CODE BEGIN OnStopJoinTimerEvent_Last */

	/* USER CODE END OnStopJoinTimerEvent_Last */
}

static void StoreContext(void) {
	LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

	/* USER CODE BEGIN StoreContext_1 */

	/* USER CODE END StoreContext_1 */
	status = LmHandlerNvmDataStore();

	if (status == LORAMAC_HANDLER_NVM_DATA_UP_TO_DATE) {
		APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA UP TO DATE\r\n");
	} else if (status == LORAMAC_HANDLER_ERROR) {
		APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORE FAILED\r\n");
	}
	/* USER CODE BEGIN StoreContext_Last */

	/* USER CODE END StoreContext_Last */
}

static void OnNvmDataChange(LmHandlerNvmContextStates_t state) {
	/* USER CODE BEGIN OnNvmDataChange_1 */

	/* USER CODE END OnNvmDataChange_1 */
	if (state == LORAMAC_HANDLER_NVM_STORE) {
		APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA STORED\r\n");
	} else {
		APP_LOG(TS_OFF, VLEVEL_M, "NVM DATA RESTORED\r\n");
	}
	/* USER CODE BEGIN OnNvmDataChange_Last */

	/* USER CODE END OnNvmDataChange_Last */
}

static void OnStoreContextRequest(void *nvm, uint32_t nvm_size) {
	/* USER CODE BEGIN OnStoreContextRequest_1 */

	/* USER CODE END OnStoreContextRequest_1 */
	/* store nvm in flash */
	if (FLASH_IF_Erase(LORAWAN_NVM_BASE_ADDRESS, FLASH_PAGE_SIZE)
			== FLASH_IF_OK) {
		FLASH_IF_Write(LORAWAN_NVM_BASE_ADDRESS, (const void*) nvm, nvm_size);
	}
	/* USER CODE BEGIN OnStoreContextRequest_Last */

	/* USER CODE END OnStoreContextRequest_Last */
}

static void OnRestoreContextRequest(void *nvm, uint32_t nvm_size) {
	/* USER CODE BEGIN OnRestoreContextRequest_1 */

	/* USER CODE END OnRestoreContextRequest_1 */
	FLASH_IF_Read(nvm, LORAWAN_NVM_BASE_ADDRESS, nvm_size);
	/* USER CODE BEGIN OnRestoreContextRequest_Last */

	/* USER CODE END OnRestoreContextRequest_Last */
}
