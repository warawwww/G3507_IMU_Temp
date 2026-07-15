#include "app.h"

#include <stdbool.h>
#include <stdint.h>

#include "KEY.h"
#include "LED.h"
#include "bsp.h"
#include "heater_task.h"
#include "host_link.h"
#include "imu_task.h"
#include "tmp_task.h"

#define APP_HEATING_LED_PERIOD_MS  (200U)
#define APP_STABLE_LED_PERIOD_MS   (1000U)
#define APP_STATE_REPORT_PERIOD_MS (500U)
#define APP_STATE_REPORT_INVALID   (0xFFU)

typedef enum {
    APP_STATE_POWER_ON_HEATING = 0,
    APP_STATE_HEATER_STABLE,
    APP_STATE_GYRO_ZERO_DRIFT,
    APP_STATE_GYRO_STABLE_TX,
    APP_STATE_360_CALIBRATION,
} App_State;

static App_State g_appState;
static bool g_heaterStartPending;
static uint32_t g_lastRedLedToggleMs;
static uint32_t g_lastAppStateReportMs;
static uint8_t g_lastReportedAppState;

static void APP_UpdateHeaterStartup(void)
{
    if (!g_heaterStartPending) {
        return;
    }

    Heater_Task_Enable();
    if (Heater_Task_IsEnabled()) {
        g_heaterStartPending = false;
    }
}

static void APP_HandleKeyEvents(void)
{
    bool rotationRequested = KEY_WasLongPressed() ||
                             HostLink_Take360CalibrationRequest();
    bool zeroRequested = KEY_WasShortPressed() ||
                         HostLink_TakeZeroCalibrationRequest();

    if (rotationRequested) {
        if (IMU_Task_Start360Calibration(1U, true)) {
            g_appState = APP_STATE_360_CALIBRATION;
        }
    } else if (zeroRequested) {
        if (IMU_Task_StartZeroCalibration(0U)) {
            g_appState = APP_STATE_GYRO_ZERO_DRIFT;
        }
    }
}

static void APP_UpdateCalibrationState(void)
{
    if ((g_appState != APP_STATE_GYRO_ZERO_DRIFT) &&
        (g_appState != APP_STATE_360_CALIBRATION)) {
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

static void APP_UpdateHeaterState(void)
{
    if ((g_appState == APP_STATE_POWER_ON_HEATING) &&
        Heater_Task_IsStable()) {
        g_appState = APP_STATE_HEATER_STABLE;
    } else if ((g_appState == APP_STATE_HEATER_STABLE) &&
        Heater_Task_IsEnabled() && !Heater_Task_IsStable()) {
        g_appState = APP_STATE_POWER_ON_HEATING;
    }
}

static void APP_UpdateState(void)
{
    APP_HandleKeyEvents();
    APP_UpdateHeaterStartup();
    APP_UpdateHeaterState();
    APP_UpdateCalibrationState();
}

static uint32_t APP_GetRedLedPeriodMs(void)
{
    return Heater_Task_IsStable() ? APP_STABLE_LED_PERIOD_MS :
        APP_HEATING_LED_PERIOD_MS;
}

static void APP_UpdateStatusLed(uint32_t nowMs)
{
    uint32_t periodMs = APP_GetRedLedPeriodMs();

    if ((uint32_t) (nowMs - g_lastRedLedToggleMs) >= periodMs) {
        g_lastRedLedToggleMs = nowMs;
        LED_Toggle(LED_ID_RED);
    }
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
    g_lastRedLedToggleMs = nowMs;
    g_lastAppStateReportMs = nowMs;
    g_lastReportedAppState = APP_STATE_REPORT_INVALID;
    g_appState = APP_STATE_POWER_ON_HEATING;
    g_heaterStartPending = true;

    KEY_Init();
    HostLink_Init();
    TMP_Task_Init();
    IMU_Task_Init();
    Heater_Task_Init();
}

void APP_Run(void)
{
    uint32_t nowMs;

    KEY_Update();
    HostLink_Run();
    TMP_Task_Run();
    IMU_Task_Run();
    APP_UpdateState();
    Heater_Task_Run();

    nowMs = BSP_GetTickMs();
    APP_ReportStateIfDue(nowMs);
    APP_UpdateStatusLed(nowMs);
}
