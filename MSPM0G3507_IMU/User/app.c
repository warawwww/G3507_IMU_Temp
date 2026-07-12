#include "app.h"

#include <stddef.h>

#include "KEY.h"
#include "LED.h"
#include "bsp.h"

#define APP_TEMPERATURE_POLL_INTERVAL_MS (100U)

static float g_temperatureC;
static bool g_temperatureValid;
static TMP117_Status g_temperatureStatus;
static uint32_t g_lastTemperaturePollMs;

static void APP_UpdateTemperature(void)
{
    uint32_t nowMs = BSP_GetTickMs();
    bool dataReady;

    if ((uint32_t) (nowMs - g_lastTemperaturePollMs) <
        APP_TEMPERATURE_POLL_INTERVAL_MS) {
        return;
    }
    g_lastTemperaturePollMs = nowMs;

    if (g_temperatureStatus != TMP117_STATUS_OK) {
        g_temperatureStatus = TMP117_Init();
        return;
    }

    g_temperatureStatus = TMP117_IsDataReady(&dataReady);
    if (g_temperatureStatus != TMP117_STATUS_OK) {
        g_temperatureValid = false;
        return;
    }

    if (dataReady) {
        g_temperatureStatus = TMP117_ReadTemperatureC(&g_temperatureC);
        g_temperatureValid  = (g_temperatureStatus == TMP117_STATUS_OK);
    }
}

void APP_Init(void)
{
    LED_Init();
    KEY_Init();

    g_temperatureC          = 0.0f;
    g_temperatureValid      = false;
    g_temperatureStatus     = TMP117_Init();
    g_lastTemperaturePollMs =
        BSP_GetTickMs() - APP_TEMPERATURE_POLL_INTERVAL_MS;
}

void APP_Run(void)
{
    KEY_Update();
    APP_UpdateTemperature();
}

bool APP_GetTemperatureC(float *temperatureC)
{
    if ((temperatureC == NULL) || !g_temperatureValid) {
        return false;
    }

    *temperatureC = g_temperatureC;
    return true;
}

TMP117_Status APP_GetTemperatureStatus(void)
{
    return g_temperatureStatus;
}
