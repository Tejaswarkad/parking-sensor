#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

// ================= USER CONFIG =================

// Battery capacity (9Ah)
#define BATTERY_CAPACITY_MAH   9000.0f

// Currents (in mA)
#define I_SLEEP   0.090f    // 90uA
#define I_SENSOR  10.0f		// 10mA
#define I_TX      150.0f    // 150mA
#define I_RX      10.0f

// Timing (seconds)
#define T_SENSOR_SEC   0.110f // ~110ms

// LoRaWAN RX windows (typical)
#define T_RX1_SEC      0.40f   // ~40 ms
#define T_RX2_SEC      0.40f   // ~40 ms

// Save threshold
#define SAVE_THRESHOLD_MAH   3.0f

// Max save interval
#define SAVE_MAX_INTERVAL_MS  864000000UL  // 10 days

// ================= API =================

void Battery_Init(void);

// Call BEFORE TX
void Battery_StartTx(void);

// Call AFTER TX (OnTxDone)
void Battery_EndTx(void);

// Battery info
uint8_t Battery_GetPercent(void);
float Battery_GetRemaining_mAh(void);

// Control
void Battery_Reset(void);
void Battery_ForceSave(void);

#endif
