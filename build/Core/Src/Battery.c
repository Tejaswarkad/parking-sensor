#include "battery.h"
#include "stm32wlxx_hal.h"
#include <string.h>
#include <math.h>
#include <main.h>
#include <sys_app.h>

// ===== External EEPROM functions =====
extern void EEPROM_Write_uint32(uint32_t addr, uint32_t data);
extern uint32_t EEPROM_Read_uint32(uint32_t addr);

// ===== Internal Variables =====
static float total_consumed_mAh = 0;
static float last_saved_mAh = 0;

static uint32_t last_event_tick = 0;
static uint32_t tx_start_tick = 0;
static uint32_t last_save_time = 0;

// ===== Internal Save =====
static void Battery_Save(void) {
	uint32_t raw;
	memcpy(&raw, &total_consumed_mAh, sizeof(float));
	EEPROM_Write_uint32(BATTERY_FLASH_ADDR, raw);

	last_saved_mAh = total_consumed_mAh;
	last_save_time = HAL_GetTick();
}

static void Battery_Save_IfNeeded(void) {
	if ((fabs(total_consumed_mAh - last_saved_mAh) >= SAVE_THRESHOLD_MAH)
			|| (HAL_GetTick() - last_save_time >= SAVE_MAX_INTERVAL_MS)) {
		Battery_Save();
	}
}

// ===== INIT =====
void Battery_Init(void) {
    uint32_t raw = EEPROM_Read_uint32(BATTERY_FLASH_ADDR);
    // 1. Check for blank/new chip (0xFFFFFFFF)
    if (raw == 0xFFFFFFFF) {
        total_consumed_mAh = 0.0f; // New battery starts at 0% consumed
    } else {
        memcpy(&total_consumed_mAh, &raw, sizeof(float));
    }
    // 2. Safety check: Reset if value is negative, too high, or an invalid number (NaN)
    if (isnan(total_consumed_mAh) || total_consumed_mAh < 0 || total_consumed_mAh > BATTERY_CAPACITY_MAH) {
        total_consumed_mAh = 0.0f;
        // Save the clean 0.0f back to flash immediately
        uint32_t clean_raw;
        memcpy(&clean_raw, &total_consumed_mAh, sizeof(float));
        EEPROM_Write_uint32(BATTERY_FLASH_ADDR, clean_raw);
    }
    last_saved_mAh = total_consumed_mAh;
    last_save_time = HAL_GetTick();

	last_event_tick = HAL_GetTick();

	// Debug print (safe integer print)
	float rem = Battery_GetRemaining_mAh();
	uint8_t per = Battery_GetPercent();

	int r_i = (int) rem;
	int r_d = (int) ((rem - r_i) * 100 + 0.5f);

    APP_LOG(TS_OFF, VLEVEL_L,
            "Battery Init: %d.%02d mAh | %d%%\n",
            r_i, r_d, per);
}

// ===== START TX =====
void Battery_StartTx(void) {
	uint32_t now = HAL_GetTick();

	// Sleep consumption since last event
	float delta_sec = (now - last_event_tick) / 1000.0f;

	float mAh_sleep = I_SLEEP * (delta_sec / 3600.0f);
	total_consumed_mAh += mAh_sleep;

	tx_start_tick = now;
}

// ===== END TX =====
void Battery_EndTx(void) {
	uint32_t now = HAL_GetTick();

	float T_tx_sec = (now - tx_start_tick) / 1000.0f;
	float T_rx_sec = T_RX1_SEC + T_RX2_SEC;

	// Energy calculation
	float mAh_sensor = I_SENSOR * (T_SENSOR_SEC / 3600.0f);
	float mAh_tx = I_TX * (T_tx_sec / 3600.0f);
	float mAh_rx = I_RX * (T_rx_sec / 3600.0f);

	total_consumed_mAh += (mAh_sensor + mAh_tx + mAh_rx);

	last_event_tick = now;

	Battery_Save_IfNeeded();
}

// ===== GETTERS =====
float Battery_GetRemaining_mAh(void) {
	float rem = BATTERY_CAPACITY_MAH - total_consumed_mAh;

	if (rem < 0)
		rem = 0;

	return rem;
}

uint8_t Battery_GetPercent(void) {
	float percent = (Battery_GetRemaining_mAh() / BATTERY_CAPACITY_MAH)
			* 100.0f;

	if (percent < 0)
		percent = 0;

	if (percent > 100)
		percent = 100;

	return (uint8_t) (percent + 0.5f);
}

// ===== RESET =====
void Battery_Reset(void) {
	total_consumed_mAh = 0;
	last_saved_mAh = 0;

	Battery_Save();

	last_event_tick = HAL_GetTick();

//    APP_LOG(TS_OFF, VLEVEL_L, "Battery Reset → 100%%\n");
}

// ===== FORCE SAVE =====
void Battery_ForceSave(void) {
	Battery_Save();
}
