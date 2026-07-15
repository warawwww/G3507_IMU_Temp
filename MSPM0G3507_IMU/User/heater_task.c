#include "heater_task.h"

#include "bsp.h"
#include "heater.h"
#include "host_link.h"
#include "pid.h"
#include "tmp_task.h"

#define HEATER_TARGET_TEMPERATURE_C       (40.0f)
/* Fixed-temperature fallback if the rate predictor underestimates inertia. */
#define HEATER_PID_ENTRY_TEMPERATURE_C    \
    (HEATER_TARGET_TEMPERATURE_C - 0.5f)
#define HEATER_OVERTEMPERATURE_C          (65.0f)
#define HEATER_MAX_DUTY_PERMILLE          (1000U)
#define HEATER_RAPID_HEAT_DUTY_PERMILLE   (1000U)
#define HEATER_FEEDFORWARD_DUTY_PERMILLE  (137U)
#define HEATER_PERMILLE_PER_PERCENT       (10.0f)
#define HEATER_MAX_DUTY_PERCENT           \
    ((float) HEATER_MAX_DUTY_PERMILLE / HEATER_PERMILLE_PER_PERCENT)
#define HEATER_FEEDFORWARD_DUTY_PERCENT   \
    ((float) HEATER_FEEDFORWARD_DUTY_PERMILLE / \
        HEATER_PERMILLE_PER_PERCENT)
#define HEATER_CONTROL_PERIOD_MS          (125U)
#define HEATER_CONTROL_PERIOD_S           (0.125f)
#define HEATER_REPORT_PERIOD_MS           (1000U)
#define HEATER_RATE_FILTER_ALPHA           (0.20f)
#define HEATER_RATE_LIMIT_C_PER_S          (2.0f)
#define HEATER_PREDICTION_LOOKAHEAD_S      (1.80f)
#define HEATER_HOLD_ENTRY_ERROR_C          (0.20f)
#define HEATER_HOLD_EXIT_ERROR_C           (0.35f)
#define HEATER_HOLD_ENTRY_RATE_C_PER_S     (0.08f)
#define HEATER_HOLD_ENTRY_SAMPLES          (16U)

/* Approach gains preserve the validated fast transient and thermal damping. */
#define HEATER_APPROACH_PID_KP             (14.0f)
#define HEATER_APPROACH_PID_KI             (2.50f)
#define HEATER_APPROACH_PID_KD             (2.50f)

/* Hold gains reject small errors quickly without exciting the slow I-cycle. */
#define HEATER_HOLD_PID_KP                 (20.0f)
#define HEATER_HOLD_PID_KI                 (1.20f)
#define HEATER_HOLD_PID_KD                 (1.00f)

#if HEATER_FEEDFORWARD_DUTY_PERMILLE > HEATER_MAX_DUTY_PERMILLE
#error "Heater feedforward duty must not exceed the maximum duty"
#endif

typedef enum {
    HEATER_PID_MODE_APPROACH = 0,
    HEATER_PID_MODE_HOLD,
} HeaterPidMode;

static PID_Controller g_approachPid;
static PID_Controller g_holdPid;
static PID_Controller *g_activePid;
static bool g_pidReady;
static bool g_pidActive;
static bool g_heaterEnabled;
static bool g_reportPending;
static bool g_controlSamplePending;
static uint32_t g_lastControlMs;
static uint32_t g_lastReportMs;
static float g_previousTemperatureC;
static float g_temperatureRateCPerS;
static float g_lastPidCorrectionPercent;
static HeaterPidMode g_pidMode;
static uint16_t g_holdEntrySampleCount;
static HostLink_HeaterControlSample g_controlSample;

static int32_t Heater_Task_RoundScaled(float value, float scale)
{
    float scaledValue = value * scale;

    return (int32_t) (scaledValue + ((scaledValue >= 0.0f) ? 0.5f : -0.5f));
}

static float Heater_Task_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void Heater_Task_UpdateTemperatureRate(float temperatureC)
{
    float rawRate =
        (temperatureC - g_previousTemperatureC) / HEATER_CONTROL_PERIOD_S;

    if (rawRate > HEATER_RATE_LIMIT_C_PER_S) {
        rawRate = HEATER_RATE_LIMIT_C_PER_S;
    } else if (rawRate < -HEATER_RATE_LIMIT_C_PER_S) {
        rawRate = -HEATER_RATE_LIMIT_C_PER_S;
    }

    g_temperatureRateCPerS += HEATER_RATE_FILTER_ALPHA *
        (rawRate - g_temperatureRateCPerS);
    g_previousTemperatureC = temperatureC;
}

static bool Heater_Task_ShouldRapidHeat(float temperatureC)
{
    float predictedTemperatureC = temperatureC;

    if (g_temperatureRateCPerS > 0.0f) {
        predictedTemperatureC += HEATER_PREDICTION_LOOKAHEAD_S *
            g_temperatureRateCPerS;
    }

    return (temperatureC < HEATER_PID_ENTRY_TEMPERATURE_C) &&
           (predictedTemperatureC < HEATER_TARGET_TEMPERATURE_C);
}

static void Heater_Task_GetActivePidGains(float *kp, float *ki, float *kd)
{
    if (g_pidActive && (g_pidMode == HEATER_PID_MODE_HOLD)) {
        *kp = HEATER_HOLD_PID_KP;
        *ki = HEATER_HOLD_PID_KI;
        *kd = HEATER_HOLD_PID_KD;
        return;
    }

    *kp = HEATER_APPROACH_PID_KP;
    *ki = HEATER_APPROACH_PID_KI;
    *kd = HEATER_APPROACH_PID_KD;
}

static bool Heater_Task_UpdatePidMode(float currentError)
{
    if (g_pidMode == HEATER_PID_MODE_APPROACH) {
        if ((Heater_Task_AbsFloat(currentError) <=
                HEATER_HOLD_ENTRY_ERROR_C) &&
            (Heater_Task_AbsFloat(g_temperatureRateCPerS) <=
                HEATER_HOLD_ENTRY_RATE_C_PER_S)) {
            if (g_holdEntrySampleCount < HEATER_HOLD_ENTRY_SAMPLES) {
                g_holdEntrySampleCount++;
            }
        } else {
            g_holdEntrySampleCount = 0U;
        }

        if (g_holdEntrySampleCount >= HEATER_HOLD_ENTRY_SAMPLES) {
            if (!PID_Prime(&g_holdPid, currentError,
                    g_lastPidCorrectionPercent)) {
                return false;
            }
            g_activePid = &g_holdPid;
            g_pidMode = HEATER_PID_MODE_HOLD;
        }
    } else if (Heater_Task_AbsFloat(currentError) >=
            HEATER_HOLD_EXIT_ERROR_C) {
        if (!PID_Prime(&g_approachPid, currentError,
                g_lastPidCorrectionPercent)) {
            return false;
        }
        g_activePid = &g_approachPid;
        g_pidMode = HEATER_PID_MODE_APPROACH;
        g_holdEntrySampleCount = 0U;
    }

    return true;
}

static void Heater_Task_QueueControlSample(uint32_t nowMs,
    HostLink_HeaterPhase phase, float temperatureC,
    float pidCorrectionPercent, uint16_t dutyPermille)
{
    float activeKp;
    float activeKi;
    float activeKd;

    Heater_Task_GetActivePidGains(&activeKp, &activeKi, &activeKd);
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
        Heater_Task_RoundScaled(activeKp, 1000.0f);
    g_controlSample.kiMilli =
        Heater_Task_RoundScaled(activeKi, 1000.0f);
    g_controlSample.kdMilli =
        Heater_Task_RoundScaled(activeKd, 1000.0f);
    g_controlSamplePending = true;
}

void Heater_Task_Disable(void)
{
    bool stateChanged =
        g_heaterEnabled || (Heater_GetDutyPermille() != 0U);

    Heater_Off();
    if (g_pidReady) {
        (void) PID_Reset(&g_approachPid, 0.0f);
        (void) PID_Reset(&g_holdPid, 0.0f);
    }

    g_heaterEnabled = false;
    g_pidActive = false;
    g_activePid = &g_approachPid;
    g_pidMode = HEATER_PID_MODE_APPROACH;
    g_holdEntrySampleCount = 0U;
    g_lastPidCorrectionPercent = 0.0f;
    g_temperatureRateCPerS = 0.0f;
    if (stateChanged) {
        g_reportPending = true;
    }
}

static void Heater_Task_EnableAt(uint32_t nowMs)
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
    (void) PID_Reset(&g_approachPid, 0.0f);
    (void) PID_Reset(&g_holdPid, 0.0f);
    g_pidActive = false;
    g_activePid = &g_approachPid;
    g_pidMode = HEATER_PID_MODE_APPROACH;
    g_holdEntrySampleCount = 0U;
    g_lastPidCorrectionPercent = 0.0f;
    g_heaterEnabled = true;
    g_reportPending = true;
    g_controlSamplePending = false;
    g_previousTemperatureC = temperatureC;
    g_temperatureRateCPerS = 0.0f;

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
    Heater_Task_UpdateTemperatureRate(temperatureC);

    if (!g_pidActive) {
        if (Heater_Task_ShouldRapidHeat(temperatureC)) {
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
        initialCorrection = HEATER_APPROACH_PID_KP * currentError;
        if (!PID_Prime(
                &g_approachPid, currentError, initialCorrection)) {
            Heater_Task_Disable();
            return;
        }
        g_activePid = &g_approachPid;
        g_pidMode = HEATER_PID_MODE_APPROACH;
        g_holdEntrySampleCount = 0U;
        g_lastPidCorrectionPercent = initialCorrection;
        g_pidActive = true;
    }

    currentError = HEATER_TARGET_TEMPERATURE_C - temperatureC;
    if (!Heater_Task_UpdatePidMode(currentError)) {
        Heater_Task_Disable();
        return;
    }

    if (!PID_Compute(g_activePid, HEATER_TARGET_TEMPERATURE_C,
            temperatureC, &pidCorrection)) {
        Heater_Task_Disable();
        return;
    }
    g_lastPidCorrectionPercent = pidCorrection;

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
    const PID_Config approachPidConfig = {
        .kp = HEATER_APPROACH_PID_KP,
        .ki = HEATER_APPROACH_PID_KI,
        .kd = HEATER_APPROACH_PID_KD,
        .samplePeriodS = HEATER_CONTROL_PERIOD_S,
        .outputMin = -(float) HEATER_FEEDFORWARD_DUTY_PERCENT,
        .outputMax = (float) (HEATER_MAX_DUTY_PERCENT -
            HEATER_FEEDFORWARD_DUTY_PERCENT),
    };
    const PID_Config holdPidConfig = {
        .kp = HEATER_HOLD_PID_KP,
        .ki = HEATER_HOLD_PID_KI,
        .kd = HEATER_HOLD_PID_KD,
        .samplePeriodS = HEATER_CONTROL_PERIOD_S,
        .outputMin = -(float) HEATER_FEEDFORWARD_DUTY_PERCENT,
        .outputMax = (float) (HEATER_MAX_DUTY_PERCENT -
            HEATER_FEEDFORWARD_DUTY_PERCENT),
    };

    Heater_Init();
    g_pidReady = PID_Init(&g_approachPid, &approachPidConfig) &&
        PID_Init(&g_holdPid, &holdPidConfig);
    g_pidActive = false;
    g_activePid = &g_approachPid;
    g_pidMode = HEATER_PID_MODE_APPROACH;
    g_holdEntrySampleCount = 0U;
    g_lastPidCorrectionPercent = 0.0f;
    g_heaterEnabled = false;
    g_reportPending = true;
    g_controlSamplePending = false;
    g_previousTemperatureC = 0.0f;
    g_temperatureRateCPerS = 0.0f;
    g_lastControlMs = BSP_GetTickMs();
    g_lastReportMs = g_lastControlMs;
}

void Heater_Task_Run(void)
{
    uint32_t nowMs = BSP_GetTickMs();

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

void Heater_Task_Enable(void)
{
    Heater_Task_EnableAt(BSP_GetTickMs());
}

bool Heater_Task_IsStable(void)
{
    return g_heaterEnabled && g_pidActive &&
        (g_pidMode == HEATER_PID_MODE_HOLD);
}

uint8_t Heater_Task_GetDutyPercent(void)
{
    return Heater_GetDutyPercent();
}

uint16_t Heater_Task_GetDutyPermille(void)
{
    return Heater_GetDutyPermille();
}
