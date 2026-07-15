#include "app.h"

#include <stdbool.h>
#include <stdint.h>

#include "KEY.h"
#include "LED.h"
#include "TMP117.h"
#include "XV7011.h"
#include "bsp.h"

#define APP_GYRO_POLL_PERIOD_MS  (20U)
#define APP_HEARTBEAT_PERIOD_MS  (500U)

volatile XV7011_Status g_gyroStatus = XV7011_STATUS_NOT_INITIALIZED;
volatile int16_t g_gyroRaw;
volatile float g_gyroDps;
volatile bool g_gyroValid;
volatile uint8_t g_gyroSpiAddressBits;

static uint32_t g_lastGyroPollMs;
static uint32_t g_lastHeartbeatMs;

void APP_Init(void)
{
    LED_Init();
    LED_On(LED_ID_RED);
    g_lastHeartbeatMs = BSP_GetTickMs();

    KEY_Init();
    (void) TMP117_Init();

    g_gyroValid      = false;
    g_gyroStatus     = XV7011_Init();
    g_gyroSpiAddressBits = XV7011_GetSpiAddressBits();
    g_lastGyroPollMs = BSP_GetTickMs();
}

void APP_Run(void)
{
    uint32_t nowMs;

    KEY_Update();

    nowMs = BSP_GetTickMs();
    if ((uint32_t) (nowMs - g_lastHeartbeatMs) >=
        APP_HEARTBEAT_PERIOD_MS) {
        g_lastHeartbeatMs = nowMs;
        LED_Toggle(LED_ID_RED);
    }

    if ((uint32_t) (nowMs - g_lastGyroPollMs) >=
        APP_GYRO_POLL_PERIOD_MS) {
        int16_t rawAngularRate;

        g_lastGyroPollMs = nowMs;
        if (g_gyroStatus == XV7011_STATUS_OK) {
            g_gyroStatus = XV7011_ReadAngularRateRaw(&rawAngularRate);
            if (g_gyroStatus == XV7011_STATUS_OK) {
                g_gyroRaw   = rawAngularRate;
                g_gyroDps   = (float) rawAngularRate /
                            XV7011_ANGULAR_RATE_LSB_PER_DPS;
                g_gyroValid = true;
            } else {
                g_gyroValid = false;
            }
        }
    }
}
