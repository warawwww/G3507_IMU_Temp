#include "heater_task.h"

#include "KEY.h"
#include "bsp.h"
#include "heater.h"
#include "host_link.h"
#include "pid.h"
#include "tmp_task.h"

#define HEATER_TARGET_TEMPERATURE_C    (50.0f)
#define HEATER_OVERTEMPERATURE_C       (70.0f)
#define HEATER_MAX_DUTY_PERCENT        (50.0f)
#define HEATER_CONTROL_PERIOD_MS       (125U)
#define HEATER_CONTROL_PERIOD_S        (0.125f)
#define HEATER_REPORT_PERIOD_MS        (1000U)

/* Conservative initial PI gains. Tune these values on the real heater. */
#define HEATER_PID_KP                   (2.0f)
#define HEATER_PID_KI                   (0.05f)
#define HEATER_PID_KD                   (0.0f)

static PID_Controller g_temperaturePid;
static bool g_pidReady;
static bool g_heaterEnabled;
static bool g_reportPending;
static uint32_t g_lastControlMs;
static uint32_t g_lastReportMs;

static void Heater_Task_Disable(void)
{
    bool stateChanged =
        g_heaterEnabled || (Heater_GetDutyPercent() != 0U);

    Heater_Off();
    if (g_pidReady) {
        (void) PID_Reset(&g_temperaturePid, 0.0f);
    }

    g_heaterEnabled = false;
    if (stateChanged) {
        g_reportPending = true;
    }
}

static void Heater_Task_Enable(uint32_t nowMs)
{
    if (!g_pidReady) {
        Heater_Task_Disable();
        return;
    }

    Heater_Off();
    (void) PID_Reset(&g_temperaturePid, 0.0f);
    g_heaterEnabled = true;
    g_reportPending = true;

    /* Force the first control calculation to run immediately. */
    g_lastControlMs = nowMs - HEATER_CONTROL_PERIOD_MS;
}

static void Heater_Task_UpdateControl(uint32_t nowMs)
{
    float temperatureC;
    float dutyPercent;
    uint8_t roundedDutyPercent;

    if (!g_heaterEnabled ||
        ((uint32_t) (nowMs - g_lastControlMs) <
            HEATER_CONTROL_PERIOD_MS)) {
        return;
    }
    g_lastControlMs = nowMs;

    if (!TMP_Task_GetTemperatureC(&temperatureC) ||
        (temperatureC >= HEATER_OVERTEMPERATURE_C)) {
        Heater_Task_Disable();
        return;
    }

    if (!PID_Compute(&g_temperaturePid, HEATER_TARGET_TEMPERATURE_C,
            temperatureC, &dutyPercent)) {
        Heater_Task_Disable();
        return;
    }

    roundedDutyPercent = (uint8_t) (dutyPercent + 0.5f);
    if (!Heater_SetDutyPercent(roundedDutyPercent)) {
        Heater_Task_Disable();
    }
}

void Heater_Task_Init(void)
{
    const PID_Config pidConfig = {
        .kp = HEATER_PID_KP,
        .ki = HEATER_PID_KI,
        .kd = HEATER_PID_KD,
        .samplePeriodS = HEATER_CONTROL_PERIOD_S,
        .outputMin = 0.0f,
        .outputMax = HEATER_MAX_DUTY_PERCENT,
    };

    Heater_Init();
    g_pidReady = PID_Init(&g_temperaturePid, &pidConfig);
    g_heaterEnabled = false;
    g_reportPending = true;
    g_lastControlMs = BSP_GetTickMs();
    g_lastReportMs = g_lastControlMs;
}

void Heater_Task_Run(void)
{
    uint32_t nowMs = BSP_GetTickMs();

    if (KEY_WasLongPressed()) {
        if (g_heaterEnabled) {
            Heater_Task_Disable();
        } else {
            Heater_Task_Enable(nowMs);
        }
    }

    Heater_Task_UpdateControl(nowMs);

    if (!g_reportPending &&
        ((uint32_t) (nowMs - g_lastReportMs) < HEATER_REPORT_PERIOD_MS)) {
        return;
    }

    if (HostLink_SendHeaterState(
            g_heaterEnabled, Heater_GetDutyPercent())) {
        g_reportPending = false;
        g_lastReportMs = nowMs;
    }
}

bool Heater_Task_IsEnabled(void)
{
    return g_heaterEnabled;
}

uint8_t Heater_Task_GetDutyPercent(void)
{
    return Heater_GetDutyPercent();
}
