#include "app.h"

#include <stdint.h>

#include "KEY.h"
#include "LED.h"
#include "bsp.h"
#include "heater_task.h"
#include "tmp_task.h"

#define APP_HEARTBEAT_PERIOD_MS (500U)

static uint32_t g_lastHeartbeatMs;

void APP_Init(void)
{
    LED_Init();
    LED_On(LED_ID_RED);
    g_lastHeartbeatMs = BSP_GetTickMs();

    KEY_Init();
    TMP_Task_Init();
    Heater_Task_Init();
}

void APP_Run(void)
{
    uint32_t nowMs;

    KEY_Update();
    TMP_Task_Run();
    Heater_Task_Run();

    nowMs = BSP_GetTickMs();
    if ((uint32_t) (nowMs - g_lastHeartbeatMs) >=
        APP_HEARTBEAT_PERIOD_MS) {
        g_lastHeartbeatMs = nowMs;
        LED_Toggle(LED_ID_RED);
    }
}
