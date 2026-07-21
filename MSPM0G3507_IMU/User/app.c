#include "app.h"

#include <stdbool.h>
#include <stdint.h>

#include "KEY.h"
#include "LED.h"
#include "app_config.h"
#include "bsp.h"
#include "heater_task.h"
#include "host_link.h"
#include "imu_task.h"
#include "tmp_task.h"

#define APP_HEATING_LED_PERIOD_MS       (200U)
#define APP_ZERO_CAL_LED_PERIOD_MS      (200U)
#define APP_STATIC_BIAS_LED_PERIOD_MS   (500U)
#define APP_STABLE_LED_PERIOD_MS        (1000U)
#define APP_STATE_REPORT_PERIOD_MS      (500U)
#define APP_STATE_REPORT_INVALID        (0xFFU)
#if APP_ENABLE_IMU_AUTOC
#define APP_STARTUP_AUTOC_RETRY_MS      (1000U)
#endif
#define APP_ROTATION_CALIBRATION_TURNS (3U)

typedef enum {
    APP_STATE_POWER_ON_HEATING = 0,
    APP_STATE_HEATER_STABLE,
    APP_STATE_GYRO_ZERO_DRIFT,
    APP_STATE_GYRO_STABLE_TX,
    APP_STATE_ROTATION_CALIBRATION,
} App_State;

static App_State g_appState;
static bool g_heaterStartPending;
#if APP_ENABLE_IMU_AUTOC
static bool g_startupHardwareZeroDone;
#endif
static bool g_startupZeroCalibrationStarted;
static bool g_redStatusLedActive;
static bool g_greenStatusLedActive;
static uint32_t g_redStatusLedPeriodMs;
static uint32_t g_greenStatusLedPeriodMs;
static uint32_t g_lastRedLedToggleMs;
static uint32_t g_lastGreenLedToggleMs;
#if APP_ENABLE_IMU_AUTOC
static uint32_t g_lastStartupHardwareZeroMs;
#endif
static uint32_t g_lastAppStateReportMs;
static uint8_t g_lastReportedAppState;

static void APP_UpdateHeaterStartup(void)
{
#if APP_ENABLE_HEATER
    if (!g_heaterStartPending) {
        return;
    }

    Heater_Task_Enable();
    if (Heater_Task_IsEnabled()) {
        g_heaterStartPending = false;
    }
#else
    g_heaterStartPending = false;
#endif
}

static void APP_HandleKeyEvents(void)
{
    bool rotationRequested = KEY_WasLongPressed() ||
                             HostLink_TakeRotationCalibrationRequest();
    bool zeroRequested = KEY_WasShortPressed() ||
                         HostLink_TakeZeroCalibrationRequest();
    bool autoCRequested = HostLink_TakeAutoCRequest();
    bool angleResetRequested = HostLink_TakeAngleResetRequest();

    if (rotationRequested) {
        if (IMU_Task_StartRotationCalibration(
                APP_ROTATION_CALIBRATION_TURNS, true)) {
            g_appState = APP_STATE_ROTATION_CALIBRATION;
        }
    } else if (autoCRequested) {
        (void) IMU_Task_RunHardwareZeroCalibration();
    } else if (zeroRequested) {
        if (IMU_Task_StartZeroCalibration(0U)) {
            g_appState = APP_STATE_GYRO_ZERO_DRIFT;
        }
    } else if (angleResetRequested) {
        (void) IMU_Task_ResetAngle();
    }
}

static void APP_UpdateCalibrationState(void)
{
    if ((g_appState != APP_STATE_GYRO_ZERO_DRIFT) &&
        (g_appState != APP_STATE_ROTATION_CALIBRATION)) {
        return;
    }

    if (IMU_Task_IsCalibrationBusy()) {
        return;
    }

    g_appState =
        (IMU_Task_GetCalibrationResult() == IMU_TASK_CAL_RESULT_OK)
            ? APP_STATE_GYRO_STABLE_TX
            : APP_STATE_HEATER_STABLE;
}

static void APP_UpdateStartupZeroCalibration(void)
{
#if APP_ENABLE_IMU_AUTOC
    uint32_t nowMs;
#endif

    if (g_startupZeroCalibrationStarted ||
        (g_appState != APP_STATE_HEATER_STABLE)) {
        return;
    }

#if APP_ENABLE_IMU_AUTOC
    if (!g_startupHardwareZeroDone) {
        nowMs = BSP_GetTickMs();
        if ((g_lastStartupHardwareZeroMs != 0U) &&
            ((uint32_t)(nowMs - g_lastStartupHardwareZeroMs) <
                APP_STARTUP_AUTOC_RETRY_MS)) {
            return;
        }

        g_lastStartupHardwareZeroMs = nowMs;
        if (!IMU_Task_RunHardwareZeroCalibration()) {
            return;
        }
        g_startupHardwareZeroDone = true;
    }
#endif

    if (IMU_Task_StartZeroCalibration(0U)) {
        g_startupZeroCalibrationStarted = true;
        g_appState = APP_STATE_GYRO_ZERO_DRIFT;
    }
}

static void APP_UpdateHeaterState(void)
{
#if APP_ENABLE_HEATER
    if ((g_appState == APP_STATE_POWER_ON_HEATING) &&
        Heater_Task_IsStable()) {
        g_appState = APP_STATE_HEATER_STABLE;
    } else if ((g_appState == APP_STATE_HEATER_STABLE) &&
        Heater_Task_IsEnabled() && !Heater_Task_IsStable()) {
        g_appState = APP_STATE_POWER_ON_HEATING;
    }
#endif
}

static void APP_UpdateState(void)
{
    APP_HandleKeyEvents();
    APP_UpdateHeaterStartup();
    APP_UpdateHeaterState();
    APP_UpdateStartupZeroCalibration();
    APP_UpdateCalibrationState();
}

static bool APP_GetRedStatusLedPeriodMs(uint32_t *periodMs)
{
#if APP_ENABLE_HEATER
    if (!Heater_Task_IsEnabled()) {
        return false;
    }

    *periodMs = Heater_Task_IsStable() ?
        APP_STABLE_LED_PERIOD_MS : APP_HEATING_LED_PERIOD_MS;
    return true;
#else
    (void)periodMs;
    return false;
#endif
}

static bool APP_GetGreenStatusLedPeriodMs(uint32_t *periodMs)
{
    if ((g_appState == APP_STATE_GYRO_ZERO_DRIFT) ||
        (g_appState == APP_STATE_ROTATION_CALIBRATION)) {
        *periodMs = APP_ZERO_CAL_LED_PERIOD_MS;
        return true;
    }

    if (g_appState == APP_STATE_GYRO_STABLE_TX) {
        if (IMU_Task_IsStaticBiasLearning()) {
            *periodMs = APP_STATIC_BIAS_LED_PERIOD_MS;
            return true;
        }

        *periodMs = APP_STABLE_LED_PERIOD_MS;
        return true;
    }

    return false;
}

static void APP_UpdateBlinkLed(LED_ID led, bool *active,
    uint32_t *lastToggleMs, uint32_t *activePeriodMs, uint32_t nowMs,
    bool enabled, uint32_t periodMs)
{
    if (!enabled) {
        if (*active) {
            LED_Off(led);
            *active = false;
        }
        return;
    }

    if (!*active || (*activePeriodMs != periodMs)) {
        LED_On(led);
        *active = true;
        *activePeriodMs = periodMs;
        *lastToggleMs = nowMs;
        return;
    }

    if ((uint32_t)(nowMs - *lastToggleMs) >= periodMs) {
        *lastToggleMs = nowMs;
        LED_Toggle(led);
    }
}

static void APP_UpdateStatusLed(uint32_t nowMs)
{
    uint32_t redPeriodMs = 0U;
    uint32_t greenPeriodMs = 0U;
    bool redEnabled = APP_GetRedStatusLedPeriodMs(&redPeriodMs);
    bool greenEnabled = APP_GetGreenStatusLedPeriodMs(&greenPeriodMs);

    APP_UpdateBlinkLed(LED_ID_RED, &g_redStatusLedActive,
        &g_lastRedLedToggleMs, &g_redStatusLedPeriodMs, nowMs,
        redEnabled, redPeriodMs);
    APP_UpdateBlinkLed(LED_ID_GREEN, &g_greenStatusLedActive,
        &g_lastGreenLedToggleMs, &g_greenStatusLedPeriodMs, nowMs,
        greenEnabled, greenPeriodMs);
}

static void APP_ReportStateIfDue(uint32_t nowMs)
{
    uint8_t appState = (uint8_t) g_appState;

    if (!HostLink_IsReportingEnabled()) {
        return;
    }

    if ((g_lastReportedAppState == appState) &&
        ((uint32_t) (nowMs - g_lastAppStateReportMs) <
            APP_STATE_REPORT_PERIOD_MS)) {
        return;
    }

    if (HostLink_SendAppState(appState)) {
        g_lastReportedAppState = appState;
        g_lastAppStateReportMs = nowMs;
    }
}

void APP_Init(void)
{
    uint32_t nowMs;

    LED_Init();
    LED_On(LED_ID_RED);
    nowMs = BSP_GetTickMs();
    g_redStatusLedActive = true;
    g_greenStatusLedActive = false;
    g_redStatusLedPeriodMs = APP_HEATING_LED_PERIOD_MS;
    g_greenStatusLedPeriodMs = APP_STABLE_LED_PERIOD_MS;
    g_lastRedLedToggleMs = nowMs;
    g_lastGreenLedToggleMs = nowMs;
#if APP_ENABLE_IMU_AUTOC
    g_lastStartupHardwareZeroMs = 0U;
#endif
    g_lastAppStateReportMs = nowMs;
    g_lastReportedAppState = APP_STATE_REPORT_INVALID;
#if APP_ENABLE_IMU_AUTOC
    g_startupHardwareZeroDone = false;
#endif
    g_startupZeroCalibrationStarted = false;
    g_appState = APP_ENABLE_HEATER ?
        APP_STATE_POWER_ON_HEATING : APP_STATE_HEATER_STABLE;
    g_heaterStartPending = APP_ENABLE_HEATER ? true : false;

    KEY_Init();
    HostLink_Init();
    TMP_Task_Init();
    IMU_Task_Init();
    Heater_Task_Init();
#if !APP_ENABLE_HEATER
    Heater_Task_Disable();
#endif
}

void APP_Run(void)
{
    uint32_t nowMs;

    KEY_Update();
    HostLink_Run();
    TMP_Task_Run();
    IMU_Task_Run();
    APP_UpdateState();
#if APP_ENABLE_HEATER
    Heater_Task_Run();
#endif

    nowMs = BSP_GetTickMs();
    APP_ReportStateIfDue(nowMs);
    APP_UpdateStatusLed(nowMs);
}
