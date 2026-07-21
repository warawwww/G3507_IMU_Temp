#ifndef USER_APP_CONFIG_H
#define USER_APP_CONFIG_H

/*
 * Set to 1 to enable the ready-state IMU static zero-hold/deadband logic.
 * Set to 0 to integrate the calibrated gyro rate directly.
 */
#define APP_ENABLE_IMU_STATIC_HOLD (1)

/*
 * Set to 1 to enable closed-loop heater startup/control.
 * Set to 0 to keep the heater output off while the rest of the app runs.
 */
#define APP_ENABLE_HEATER (1)

/*
 * Set to 1 to run XV7021BB internal AutoC once after temperature stabilizes
 * and before the software zero-drift calibration starts.
 * Set to 0 to use only the software zero-drift calibration.
 */
#define APP_ENABLE_IMU_AUTOC (1)

#if (APP_ENABLE_IMU_STATIC_HOLD != 0) && (APP_ENABLE_IMU_STATIC_HOLD != 1)
#error "APP_ENABLE_IMU_STATIC_HOLD must be 0 or 1"
#endif

#if (APP_ENABLE_HEATER != 0) && (APP_ENABLE_HEATER != 1)
#error "APP_ENABLE_HEATER must be 0 or 1"
#endif

#if (APP_ENABLE_IMU_AUTOC != 0) && (APP_ENABLE_IMU_AUTOC != 1)
#error "APP_ENABLE_IMU_AUTOC must be 0 or 1"
#endif

#endif
