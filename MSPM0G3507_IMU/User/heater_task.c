#include "heater_task.h"

#include "KEY.h"
#include "bsp.h"
#include "heater.h"
#include "host_link.h"
#include "pid.h"
#include "tmp_task.h"

#define HEATER_TARGET_TEMPERATURE_C       (50.0f)
#define HEATER_PID_ENTRY_TEMPERATURE_C    (49.5f)
#define HEATER_OVERTEMPERATURE_C          (65.0f)
#define HEATER_MAX_DUTY_PERMILLE          (1000U)
#define HEATER_RAPID_HEAT_DUTY_PERMILLE   (1000U)
#define HEATER_FEEDFORWARD_DUTY_PERMILLE  (250U)
#define HEATER_PERMILLE_PER_PERCENT       (10.0f)
#define HEATER_MAX_DUTY_PERCENT           \
    ((float) HEATER_MAX_DUTY_PERMILLE / HEATER_PERMILLE_PER_PERCENT)
#define HEATER_FEEDFORWARD_DUTY_PERCENT   \
    ((float) HEATER_FEEDFORWARD_DUTY_PERMILLE / \
        HEATER_PERMILLE_PER_PERCENT)
#define HEATER_CONTROL_PERIOD_MS          (125U)
#define HEATER_CONTROL_PERIOD_S           (0.125f)
#define HEATER_REPORT_PERIOD_MS           (1000U)

/* Initial gains for a faster response with derivative damping. */
#define HEATER_PID_KP                   (10.0f)
#define HEATER_PID_KI                   (0.60f)
#define HEATER_PID_KD                   (1.0f)

#if HEATER_FEEDFORWARD_DUTY_PERMILLE > HEATER_MAX_DUTY_PERMILLE
#error "Heater feedforward duty must not exceed the maximum duty"
#endif

static PID_Controller g_temperaturePid;
static bool g_pidReady;
static bool g_pidActive;
static bool g_heaterEnabled;
static bool g_reportPending;
static bool g_controlSamplePending;
static uint32_t g_lastControlMs;
static uint32_t g_lastReportMs;
static HostLink_HeaterControlSample g_controlSample;

static int32_t Heater_Task_RoundScaled(float value, float scale)
{
    float scaledValue = value * scale;

    return (int32_t) (scaledValue + ((scaledValue >= 0.0f) ? 0.5f : -0.5f));
}

static void Heater_Task_QueueControlSample(uint32_t nowMs,
    HostLink_HeaterPhase phase, float temperatureC,
    float pidCorrectionPercent, uint16_t dutyPermille)
{
    g_controlSample.timeMs = nowMs;
    g_controlSample.phase = phase;
    g_controlSample.temperatureMilliC =
        Heater_Task_RoundScaled(temperatureC, 1000.0f);
    g_controlSample.targetMilliC =
        Heater_Task_RoundScaled(HEATER_TARGET_TEMPERATURE_C, 1000.0f);
    g_controlSample.dutyPermille = dutyPermille;
    g_controlSample.feedforwardPermille =
        HEATER_FEEDFORWARD_DUTY_PERMILLE;
    g_controlSample.pidCorrectionPermille =
        Heater_Task_RoundScaled(pidCorrectionPercent,
            HEATER_PERMILLE_PER_PERCENT);
    g_controlSample.kpMilli =
        Heater_Task_RoundScaled(HEATER_PID_KP, 1000.0f);
    g_controlSample.kiMilli =
        Heater_Task_RoundScaled(HEATER_PID_KI, 1000.0f);
    g_controlSample.kdMilli =
        Heater_Task_RoundScaled(HEATER_PID_KD, 1000.0f);
    g_controlSamplePending = true;
}

static void Heater_Task_Disable(void)
{
    bool stateChanged =
        g_heaterEnabled || (Heater_GetDutyPermille() != 0U);

    Heater_Off();
    if (g_pidReady) {
        (void) PID_Reset(&g_temperaturePid, 0.0f);
    }

    g_heaterEnabled = false;
    g_pidActive = false;
    if (stateChanged) {
        g_reportPending = true;
    }
}

static void Heater_Task_Enable(uint32_t nowMs)
{
    float temperatureC;

    if (!g_pidReady) {
        Heater_Task_Disable();
        return;
    }

    if (!TMP_Task_GetTemperatureC(&temperatureC) ||
        (temperatureC >= HEATER_OVERTEMPERATURE_C)) {
        Heater_Task_Disable();
        return;
    }

    Heater_Off();
    (void) PID_Reset(&g_temperaturePid, 0.0f);
    g_pidActive = false;
    g_heaterEnabled = true;
    g_reportPending = true;
    g_controlSamplePending = false;

    /* Force the first control calculation to run immediately. */
    g_lastControlMs = nowMs - HEATER_CONTROL_PERIOD_MS;
}

static void Heater_Task_UpdateControl(uint32_t nowMs)
{
    float temperatureC;
    float pidCorrection;
    float dutyPercent;
    float currentError;
    float initialCorrection;
    uint16_t dutyPermille;

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

    if (!g_pidActive) {
        if (temperatureC < HEATER_PID_ENTRY_TEMPERATURE_C) {
            if (!Heater_SetDutyPermille(
                    HEATER_RAPID_HEAT_DUTY_PERMILLE)) {
                Heater_Task_Disable();
                return;
            }
            Heater_Task_QueueControlSample(nowMs,
                HOST_LINK_HEATER_PHASE_RAPID, temperatureC, 0.0f,
                HEATER_RAPID_HEAT_DUTY_PERMILLE);
            return;
        }

        currentError = HEATER_TARGET_TEMPERATURE_C - temperatureC;
        initialCorrection = HEATER_PID_KP * currentError;
        if (!PID_Prime(
                &g_temperaturePid, currentError, initialCorrection)) {
            Heater_Task_Disable();
            return;
        }
        g_pidActive = true;
    }

    if (!PID_Compute(&g_temperaturePid, HEATER_TARGET_TEMPERATURE_C,
            temperatureC, &pidCorrection)) {
        Heater_Task_Disable();
        return;
    }

    dutyPercent = (float) HEATER_FEEDFORWARD_DUTY_PERCENT + pidCorrection;
    if (dutyPercent < 0.0f) {
        dutyPercent = 0.0f;
    } else if (dutyPercent > HEATER_MAX_DUTY_PERCENT) {
        dutyPercent = HEATER_MAX_DUTY_PERCENT;
    }

    dutyPermille =
        (uint16_t) (dutyPercent * HEATER_PERMILLE_PER_PERCENT + 0.5f);
    if (dutyPermille > HEATER_MAX_DUTY_PERMILLE) {
        dutyPermille = HEATER_MAX_DUTY_PERMILLE;
    }
    if (!Heater_SetDutyPermille(dutyPermille)) {
        Heater_Task_Disable();
        return;
    }
    Heater_Task_QueueControlSample(nowMs, HOST_LINK_HEATER_PHASE_PID,
        temperatureC, pidCorrection, dutyPermille);
}

void Heater_Task_Init(void)
{
    const PID_Config pidConfig = {
        .kp = HEATER_PID_KP,
        .ki = HEATER_PID_KI,
        .kd = HEATER_PID_KD,
        .samplePeriodS = HEATER_CONTROL_PERIOD_S,
        .outputMin = -(float) HEATER_FEEDFORWARD_DUTY_PERCENT,
        .outputMax = (float) (HEATER_MAX_DUTY_PERCENT -
            HEATER_FEEDFORWARD_DUTY_PERCENT),
    };

    Heater_Init();
    g_pidReady = PID_Init(&g_temperaturePid, &pidConfig);
    g_pidActive = false;
    g_heaterEnabled = false;
    g_reportPending = true;
    g_controlSamplePending = false;
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

    /* TMP117 reports first and may briefly own the DMA TX buffer. Retry the
     * synchronized control sample on subsequent main-loop iterations. */
    if (g_controlSamplePending) {
        if (HostLink_SendHeaterControlSample(&g_controlSample)) {
            g_controlSamplePending = false;
        }
        return;
    }

    if (!g_reportPending &&
        ((uint32_t) (nowMs - g_lastReportMs) < HEATER_REPORT_PERIOD_MS)) {
        return;
    }

    if (HostLink_SendHeaterState(
            g_heaterEnabled, Heater_GetDutyPermille())) {
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

uint16_t Heater_Task_GetDutyPermille(void)
{
    return Heater_GetDutyPermille();
}
