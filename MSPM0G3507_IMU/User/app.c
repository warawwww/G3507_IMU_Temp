#include "app.h"

#include <stdbool.h>
#include <stdint.h>

#include "KEY.h"
#include "LED.h"
#include "bsp.h"
#include "heater_task.h"
#include "tmp_task.h"

#define APP_HEATING_LED_PERIOD_MS (200U)
#define APP_STABLE_LED_PERIOD_MS  (1000U)

/*
 * 应用层业务状态机。
 *
 * heater_task 只负责温度闭环和必要保护；按键语义、陀螺仪业务流程、
 * 标定流程等上层调度都集中放在这里。
 */
typedef enum {
    /* 上电后的默认状态：等待 TMP 温度有效后开启加热，并持续升温。 */
    APP_STATE_POWER_ON_HEATING = 0,

    /* 加热闭环进入保持阶段，温度已经稳定，可进入后续陀螺仪业务。 */
    APP_STATE_HEATER_STABLE,

    /* 短按按键进入：后续在这里执行陀螺仪零飘采集和补偿。 */
    APP_STATE_GYRO_ZERO_DRIFT,

    /* 零飘处理完成后的正常工作态：后续在这里稳定读取并传输陀螺仪数据。 */
    APP_STATE_GYRO_STABLE_TX,

    /* 长按按键进入：后续在这里执行 360 度标定流程。 */
    APP_STATE_360_CALIBRATION,
} App_State;

static App_State g_appState;

/* 上电默认需要加热；若第一帧 TMP 还没读到，则保持 pending 后续重试。 */
static bool g_heaterStartPending;
static uint32_t g_lastRedLedToggleMs;

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

static void APP_UpdateState(void)
{
    /* 按键只在 app 层消费，避免底层任务私自改变业务状态。 */
    if (KEY_WasLongPressed()) {
        g_appState = APP_STATE_360_CALIBRATION;
    } else if (KEY_WasShortPressed()) {
        g_appState = APP_STATE_GYRO_ZERO_DRIFT;
    }

    APP_UpdateHeaterStartup();

    /* 加热稳定与否由 heater_task 判断，app 只根据结果切换业务态。 */
    if ((g_appState == APP_STATE_POWER_ON_HEATING) &&
        Heater_Task_IsStable()) {
        g_appState = APP_STATE_HEATER_STABLE;
    } else if ((g_appState == APP_STATE_HEATER_STABLE) &&
        Heater_Task_IsEnabled() && !Heater_Task_IsStable()) {
        g_appState = APP_STATE_POWER_ON_HEATING;
    }
}

static uint32_t APP_GetRedLedPeriodMs(void)
{
    /* 红灯直接指示温控状态：加热阶段快闪，稳定阶段慢闪。 */
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

void APP_Init(void)
{
    uint32_t nowMs;

    LED_Init();
    LED_On(LED_ID_RED);
    nowMs = BSP_GetTickMs();
    g_lastRedLedToggleMs = nowMs;
    g_appState = APP_STATE_POWER_ON_HEATING;
    g_heaterStartPending = true;

    KEY_Init();
    TMP_Task_Init();
    Heater_Task_Init();
}

void APP_Run(void)
{
    uint32_t nowMs;

    KEY_Update();
    TMP_Task_Run();
    APP_UpdateState();
    Heater_Task_Run();

    nowMs = BSP_GetTickMs();
    APP_UpdateStatusLed(nowMs);
}
