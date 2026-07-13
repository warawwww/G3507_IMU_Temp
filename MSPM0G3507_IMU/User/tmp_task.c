#include "tmp_task.h"

#include <stddef.h>
#include <stdint.h>

#include "bsp.h"
#include "host_link.h"

#define TMP_TASK_SAMPLE_PERIOD_MS (125U)
#define TMP_TASK_RETRY_PERIOD_MS  (1000U)

static uint32_t g_lastActionMs;

static volatile TMP117_Status g_tmp117Status = TMP117_STATUS_NOT_FOUND;
static volatile float g_tmp117TemperatureC;
static volatile bool g_tmp117Valid;

static void TMP_Task_InitializeSensor(void)
{
    g_tmp117Valid  = false;
    g_tmp117Status = TMP117_Init();

    if (g_tmp117Status != TMP117_STATUS_OK) {
        (void) HostLink_SendTMP117Error(g_tmp117Status);
    }
}

static void TMP_Task_ReadAndReport(void)
{
    int16_t rawTemperature;

    g_tmp117Status = TMP117_ReadRaw(&rawTemperature);
    if (g_tmp117Status != TMP117_STATUS_OK) {
        g_tmp117Valid = false;
        (void) HostLink_SendTMP117Error(g_tmp117Status);
        return;
    }

    g_tmp117TemperatureC =
        (float) rawTemperature * TMP117_TEMPERATURE_LSB_C;
    g_tmp117Valid = true;
    (void) HostLink_SendTMP117TemperatureRaw(rawTemperature);
}

void TMP_Task_Init(void)
{
    g_lastActionMs = BSP_GetTickMs();
    TMP_Task_InitializeSensor();
}

void TMP_Task_Run(void)
{
    uint32_t nowMs = BSP_GetTickMs();
    uint32_t periodMs = (g_tmp117Status == TMP117_STATUS_OK)
                            ? TMP_TASK_SAMPLE_PERIOD_MS
                            : TMP_TASK_RETRY_PERIOD_MS;

    if ((uint32_t) (nowMs - g_lastActionMs) < periodMs) {
        return;
    }
    g_lastActionMs = nowMs;

    if (g_tmp117Status != TMP117_STATUS_OK) {
        TMP_Task_InitializeSensor();
        return;
    }

    TMP_Task_ReadAndReport();
}

bool TMP_Task_GetTemperatureC(float *temperatureC)
{
    if ((temperatureC == NULL) || !g_tmp117Valid) {
        return false;
    }

    *temperatureC = g_tmp117TemperatureC;
    return true;
}

TMP117_Status TMP_Task_GetStatus(void)
{
    return g_tmp117Status;
}
