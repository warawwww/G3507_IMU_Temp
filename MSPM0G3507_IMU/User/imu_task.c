#include "imu_task.h"

#include <stddef.h>

#include "app_config.h"
#include "bsp.h"
#include "host_link.h"
#include "imu_cal_storage.h"

#define IMU_TASK_SAMPLE_PERIOD_US           (200U)
#define IMU_TASK_RETRY_PERIOD_MS            (1000U)
#define IMU_TASK_TEMPERATURE_PERIOD_MS      (50U)
#define IMU_TASK_REPORT_PERIOD_MS           (10U)
#define IMU_TASK_ZERO_CAL_SETTLE_MS         (1000U)
#define IMU_TASK_DEFAULT_ZERO_CAL_MS        (3000U)
#define IMU_TASK_MIN_ZERO_CAL_MS            (500U)
#define IMU_TASK_MAX_ZERO_CAL_MS            (30000U)
#define IMU_TASK_STATIC_MIN_SAMPLES         (64U)
#define IMU_TASK_STATIC_MAX_ABS_DPS         (5.0f)
#define IMU_TASK_STATIC_VARIANCE_LIMIT_DPS2 (0.25f)
#define IMU_TASK_ROTATION_STATIC_MS         (1000U)
#define IMU_TASK_ROTATION_TIMEOUT_MS        (60000U)
#define IMU_TASK_ROTATION_STILL_RATE_DPS    (2.0f)
#define IMU_TASK_ROTATION_START_RATE_DPS    (5.0f)
#define IMU_TASK_ROTATION_MIN_TARGET_RATIO  (0.80f)
#define IMU_TASK_SCALE_CORRECTION_MIN       (0.80f)
#define IMU_TASK_SCALE_CORRECTION_MAX       (1.20f)
#define IMU_TASK_SATURATION_RATE_DPS        (380.0f)
#define IMU_TASK_STATIC_HOLD_WINDOW_MS      (200U)
#define IMU_TASK_STATIC_HOLD_ENTER_MEAN_ABS_DPS \
    (0.025f)
#define IMU_TASK_STATIC_HOLD_ENTER_VARIANCE_DPS2 \
    (0.001225f)
#define IMU_TASK_STATIC_HOLD_ENTER_MAX_ABS_DPS \
    (0.10f)
#define IMU_TASK_STATIC_HOLD_EXIT_RATE_DPS (0.12f)
#define IMU_TASK_STATIC_HOLD_EXIT_MEAN_ABS_DPS \
    (0.06f)
#define IMU_TASK_STATIC_BIAS_ADAPT_DELAY_MS     (3000U)
#define IMU_TASK_STATIC_BIAS_ADAPT_ALPHA_MIN    (0.005f)
#define IMU_TASK_STATIC_BIAS_ADAPT_ALPHA_MAX    (0.025f)
#define IMU_TASK_STATIC_BIAS_ADAPT_RAMP_MS      (10000U)
#define IMU_TASK_STATIC_BIAS_ADAPT_MAX_STEP_DPS (0.002f)
#define IMU_TASK_STATIC_BIAS_ADAPT_MAX_TOTAL_DPS \
    (0.10f)
#define IMU_TASK_ROTATION_CALIBRATION_LEGS (2U)
#define IMU_TASK_MILLI_SCALE (1000.0f)
#define IMU_TASK_PPM_SCALE   (1000000.0f)

typedef enum {
    IMU_TASK_ROTATION_PHASE_IDLE = 0,
    IMU_TASK_ROTATION_PHASE_PRE_STATIC,
    IMU_TASK_ROTATION_PHASE_ROTATING,
} IMU_Task_RotationPhase;

typedef struct {
    uint32_t count;
    float meanRaw24;
    float meanDps;
    float m2Dps;
    float maxAbsDps;
} IMU_Task_Stats;

#if APP_ENABLE_IMU_STATIC_HOLD
typedef struct {
    uint32_t startMs;
    uint32_t count;
    float sumRateDps;
    float sumRateDps2;
    float sumAbsRateDps;
    float maxAbsRateDps;
    uint32_t biasAdaptStartMs;
    float biasAdaptBaseDeltaRaw24;
    bool holding;
    bool biasAdaptActive;
} IMU_Task_StaticHold;
#endif

static IMU_Task_State g_state;
static IMU_Task_CalibrationResult g_calibrationResult;
static XV7021_Status g_sensorStatus;
static IMU_Task_Sample g_sample;

static uint32_t g_lastSampleMs;
static uint32_t g_lastSampleUs;
static uint32_t g_lastInitAttemptMs;
static uint32_t g_lastTemperatureMs;
static uint32_t g_lastReportMs;
static uint32_t g_zeroCalibrationMs;
static uint32_t g_calibrationStartMs;
static uint32_t g_phaseStartMs;
static uint32_t g_rotationStillStartMs;
static uint16_t g_rotationTurns;
static bool g_rotationClockwise;
static bool g_rotationSeen;
static uint8_t g_rotationLegIndex;
static float g_rotationFirstAngleDeg;
static float g_rotationRegressionNumerator;
static float g_rotationRegressionDenominator;
static bool g_zeroCalibrationCollecting;
static bool g_reportPending;

static float g_baseBiasRaw24;
static float g_adaptiveBiasDeltaRaw24;
static float g_scaleCorrection;
static float g_previousRateDps;
static bool g_previousRateValid;
static float g_rotationAngleDeg;
static float g_rotationPreviousRateDps;
static bool g_rotationPreviousRateValid;
static bool g_rotationSaturated;
static uint32_t g_rotationLastUs;
static IMU_Task_RotationPhase g_rotationPhase;
static IMU_Task_Stats g_stats;
#if APP_ENABLE_IMU_STATIC_HOLD
static IMU_Task_StaticHold g_staticHold;
#endif

static float IMU_Task_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float IMU_Task_ClampFloat(float value, float minValue,
                                 float maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }

    return value;
}

static int32_t IMU_Task_RoundScaled(float value, float scale)
{
    float scaledValue = value * scale;

    return (int32_t)(scaledValue + ((scaledValue >= 0.0f) ? 0.5f : -0.5f));
}

static float IMU_Task_Raw24ToDps(float rawAngularRate24)
{
    return rawAngularRate24 / XV7021_ANGULAR_RATE_24BIT_LSB_PER_DPS;
}

static float IMU_Task_GetEffectiveBiasRaw24(void)
{
    return g_baseBiasRaw24 + g_adaptiveBiasDeltaRaw24;
}

static void IMU_Task_ResetAdaptiveBias(void)
{
    g_adaptiveBiasDeltaRaw24 = 0.0f;
}

static void IMU_Task_SetBaseBiasRaw24(float biasRaw24)
{
    g_baseBiasRaw24 = biasRaw24;
    IMU_Task_ResetAdaptiveBias();
}

static float IMU_Task_NormalizeAngleDeg(float angleDeg)
{
    while (angleDeg >= 360.0f) {
        angleDeg -= 360.0f;
    }
    while (angleDeg < 0.0f) {
        angleDeg += 360.0f;
    }

    return angleDeg;
}

static bool IMU_Task_IsScaleCorrectionValid(float scaleCorrection)
{
    return (scaleCorrection >= IMU_TASK_SCALE_CORRECTION_MIN) &&
           (scaleCorrection <= IMU_TASK_SCALE_CORRECTION_MAX);
}

static void IMU_Task_LoadStoredCalibration(void)
{
    IMU_CalStorage_Data storedCalibration;

    if (!IMU_CalStorage_Load(&storedCalibration)) {
        return;
    }
    if (!IMU_Task_IsScaleCorrectionValid(
            storedCalibration.scaleCorrection)) {
        return;
    }

    IMU_Task_SetBaseBiasRaw24(storedCalibration.biasRaw24);
    g_scaleCorrection = storedCalibration.scaleCorrection;
}

static void IMU_Task_SaveStoredCalibration(uint32_t sampleCount)
{
    IMU_CalStorage_Data calibration;

    calibration.biasRaw24        = g_baseBiasRaw24;
    calibration.scaleCorrection  = g_scaleCorrection;
    calibration.biasTemperatureC = g_sample.temperatureC;
    calibration.sampleCount      = sampleCount;

    (void)IMU_CalStorage_Save(&calibration);
}

static void IMU_Task_ResetStats(void)
{
    g_stats.count     = 0U;
    g_stats.meanRaw24 = 0.0f;
    g_stats.meanDps   = 0.0f;
    g_stats.m2Dps     = 0.0f;
    g_stats.maxAbsDps = 0.0f;
}

#if APP_ENABLE_IMU_STATIC_HOLD
static void IMU_Task_ResetStaticHoldWindow(uint32_t nowMs)
{
    g_staticHold.startMs       = nowMs;
    g_staticHold.count         = 0U;
    g_staticHold.sumRateDps    = 0.0f;
    g_staticHold.sumRateDps2   = 0.0f;
    g_staticHold.sumAbsRateDps = 0.0f;
    g_staticHold.maxAbsRateDps = 0.0f;
}

static void IMU_Task_ResetStaticBiasAdapt(void)
{
    g_staticHold.biasAdaptStartMs      = 0U;
    g_staticHold.biasAdaptBaseDeltaRaw24 = g_adaptiveBiasDeltaRaw24;
    g_staticHold.biasAdaptActive       = false;
}

static void IMU_Task_PauseStaticBiasAdapt(void)
{
    g_staticHold.biasAdaptStartMs = 0U;
    g_staticHold.biasAdaptActive  = false;
}

static void IMU_Task_StartStaticBiasAdapt(uint32_t nowMs)
{
    g_staticHold.biasAdaptStartMs   = nowMs;
    g_staticHold.biasAdaptActive    = true;
}

static float IMU_Task_RateDpsToBiasRaw24(float rateDps)
{
    float scaleCorrection = g_scaleCorrection;

    if (!IMU_Task_IsScaleCorrectionValid(scaleCorrection)) {
        scaleCorrection = 1.0f;
    }

    return (rateDps * XV7021_ANGULAR_RATE_24BIT_LSB_PER_DPS) /
           scaleCorrection;
}

static float IMU_Task_GetBiasAdaptConfidence(float value, float limit)
{
    if (limit <= 0.0f) {
        return 0.0f;
    }

    return IMU_Task_ClampFloat(1.0f - (value / limit), 0.0f, 1.0f);
}

static float IMU_Task_GetStaticBiasAdaptAlpha(uint32_t nowMs,
                                              float meanAbsRateDps,
                                              float varianceDps2,
                                              float maxAbsRateDps)
{
    uint32_t elapsedMs;
    uint32_t rampMs;
    float meanConfidence;
    float varianceConfidence;
    float maxConfidence;
    float qualityConfidence;
    float timeConfidence;
    float confidence;

    elapsedMs = (uint32_t)(nowMs - g_staticHold.biasAdaptStartMs);
    if (elapsedMs <= IMU_TASK_STATIC_BIAS_ADAPT_DELAY_MS) {
        return IMU_TASK_STATIC_BIAS_ADAPT_ALPHA_MIN;
    }

    rampMs = elapsedMs - IMU_TASK_STATIC_BIAS_ADAPT_DELAY_MS;
    timeConfidence = IMU_Task_ClampFloat(
        (float)rampMs / (float)IMU_TASK_STATIC_BIAS_ADAPT_RAMP_MS,
        0.0f, 1.0f);

    meanConfidence = IMU_Task_GetBiasAdaptConfidence(
        meanAbsRateDps, IMU_TASK_STATIC_HOLD_ENTER_MEAN_ABS_DPS);
    varianceConfidence = IMU_Task_GetBiasAdaptConfidence(
        varianceDps2, IMU_TASK_STATIC_HOLD_ENTER_VARIANCE_DPS2);
    maxConfidence = IMU_Task_GetBiasAdaptConfidence(
        maxAbsRateDps, IMU_TASK_STATIC_HOLD_ENTER_MAX_ABS_DPS);

    qualityConfidence = meanConfidence;
    if (varianceConfidence < qualityConfidence) {
        qualityConfidence = varianceConfidence;
    }
    if (maxConfidence < qualityConfidence) {
        qualityConfidence = maxConfidence;
    }

    confidence = qualityConfidence * timeConfidence;
    return IMU_TASK_STATIC_BIAS_ADAPT_ALPHA_MIN +
           (IMU_TASK_STATIC_BIAS_ADAPT_ALPHA_MAX -
            IMU_TASK_STATIC_BIAS_ADAPT_ALPHA_MIN) * confidence;
}

static void IMU_Task_UpdateStaticBiasAdapt(uint32_t nowMs,
                                           float meanRateDps,
                                           float meanAbsRateDps,
                                           float varianceDps2,
                                           float maxAbsRateDps)
{
    float alpha;
    float stepRaw24;
    float maxStepRaw24;
    float maxTotalRaw24;
    float currentDeltaRaw24;

    if (!g_staticHold.biasAdaptActive) {
        IMU_Task_StartStaticBiasAdapt(nowMs);
        return;
    }

    if ((uint32_t)(nowMs - g_staticHold.biasAdaptStartMs) <
        IMU_TASK_STATIC_BIAS_ADAPT_DELAY_MS) {
        return;
    }

    alpha = IMU_Task_GetStaticBiasAdaptAlpha(
        nowMs, meanAbsRateDps, varianceDps2, maxAbsRateDps);
    stepRaw24 = IMU_Task_RateDpsToBiasRaw24(meanRateDps) *
                alpha;
    maxStepRaw24 = IMU_Task_AbsFloat(IMU_Task_RateDpsToBiasRaw24(
        IMU_TASK_STATIC_BIAS_ADAPT_MAX_STEP_DPS));
    maxTotalRaw24 = IMU_Task_AbsFloat(IMU_Task_RateDpsToBiasRaw24(
        IMU_TASK_STATIC_BIAS_ADAPT_MAX_TOTAL_DPS));

    stepRaw24 = IMU_Task_ClampFloat(stepRaw24, -maxStepRaw24,
                                    maxStepRaw24);
    currentDeltaRaw24 =
        (g_adaptiveBiasDeltaRaw24 -
         g_staticHold.biasAdaptBaseDeltaRaw24) + stepRaw24;
    currentDeltaRaw24 = IMU_Task_ClampFloat(
        currentDeltaRaw24, -maxTotalRaw24, maxTotalRaw24);

    g_adaptiveBiasDeltaRaw24 =
        g_staticHold.biasAdaptBaseDeltaRaw24 + currentDeltaRaw24;
}

static void IMU_Task_ResetStaticHold(uint32_t nowMs)
{
    g_staticHold.holding = false;
    IMU_Task_ResetStaticBiasAdapt();
    IMU_Task_ResetStaticHoldWindow(nowMs);
}

static void IMU_Task_EnterStaticHold(uint32_t nowMs)
{
    g_staticHold.holding               = true;
    g_staticHold.biasAdaptBaseDeltaRaw24 = g_adaptiveBiasDeltaRaw24;
    IMU_Task_StartStaticBiasAdapt(nowMs);
}

static void IMU_Task_UpdateStaticHoldWindow(float rateDps)
{
    float absRateDps = IMU_Task_AbsFloat(rateDps);

    g_staticHold.count++;
    g_staticHold.sumRateDps += rateDps;
    g_staticHold.sumRateDps2 += rateDps * rateDps;
    g_staticHold.sumAbsRateDps += absRateDps;
    if (absRateDps > g_staticHold.maxAbsRateDps) {
        g_staticHold.maxAbsRateDps = absRateDps;
    }
}

static float IMU_Task_ApplyStaticHold(uint32_t nowMs, float rateDps)
{
    float absRateDps = IMU_Task_AbsFloat(rateDps);

    if (g_staticHold.holding &&
        (absRateDps > IMU_TASK_STATIC_HOLD_EXIT_RATE_DPS)) {
        g_staticHold.holding = false;
        IMU_Task_ResetStaticBiasAdapt();
        IMU_Task_ResetStaticHoldWindow(nowMs);
    }

    IMU_Task_UpdateStaticHoldWindow(rateDps);

    if ((uint32_t)(nowMs - g_staticHold.startMs) >=
        IMU_TASK_STATIC_HOLD_WINDOW_MS) {
        float count          = (float)g_staticHold.count;
        float meanRateDps    = g_staticHold.sumRateDps / count;
        float meanAbsRateDps = g_staticHold.sumAbsRateDps / count;
        bool stableWindow;
        float varianceDps2 =
            (g_staticHold.sumRateDps2 / count) -
            (meanRateDps * meanRateDps);

        if (varianceDps2 < 0.0f) {
            varianceDps2 = 0.0f;
        }

        stableWindow =
            (meanAbsRateDps < IMU_TASK_STATIC_HOLD_ENTER_MEAN_ABS_DPS) &&
            (varianceDps2 < IMU_TASK_STATIC_HOLD_ENTER_VARIANCE_DPS2) &&
            (g_staticHold.maxAbsRateDps <
             IMU_TASK_STATIC_HOLD_ENTER_MAX_ABS_DPS);

        if (g_staticHold.holding) {
            if (meanAbsRateDps >
                IMU_TASK_STATIC_HOLD_EXIT_MEAN_ABS_DPS) {
                g_staticHold.holding = false;
                IMU_Task_ResetStaticBiasAdapt();
            } else if (stableWindow) {
                IMU_Task_UpdateStaticBiasAdapt(
                    nowMs, meanRateDps, meanAbsRateDps, varianceDps2,
                    g_staticHold.maxAbsRateDps);
            } else {
                IMU_Task_PauseStaticBiasAdapt();
            }
        } else if (stableWindow) {
            IMU_Task_EnterStaticHold(nowMs);
        }

        IMU_Task_ResetStaticHoldWindow(nowMs);
    }

    return g_staticHold.holding ? 0.0f : rateDps;
}
#else
static void IMU_Task_ResetStaticHold(uint32_t nowMs)
{
    (void)nowMs;
}
#endif

static void IMU_Task_UpdateStats(int32_t rawAngularRate24, float rawDps)
{
    float count;
    float deltaRaw;
    float deltaDps;
    float absDps;

    g_stats.count++;
    count = (float)g_stats.count;

    deltaRaw = (float)rawAngularRate24 - g_stats.meanRaw24;
    g_stats.meanRaw24 += deltaRaw / count;

    deltaDps = rawDps - g_stats.meanDps;
    g_stats.meanDps += deltaDps / count;
    g_stats.m2Dps += deltaDps * (rawDps - g_stats.meanDps);

    absDps = IMU_Task_AbsFloat(rawDps);
    if (absDps > g_stats.maxAbsDps) {
        g_stats.maxAbsDps = absDps;
    }
}

static float IMU_Task_GetStatsVarianceDps2(void)
{
    if (g_stats.count < 2U) {
        return 0.0f;
    }

    return g_stats.m2Dps / (float)(g_stats.count - 1U);
}

static bool IMU_Task_AreStatsStatic(void)
{
    if (g_stats.count < IMU_TASK_STATIC_MIN_SAMPLES) {
        return false;
    }

    return (g_stats.maxAbsDps <= IMU_TASK_STATIC_MAX_ABS_DPS) &&
           (IMU_Task_GetStatsVarianceDps2() <=
            IMU_TASK_STATIC_VARIANCE_LIMIT_DPS2);
}

static void IMU_Task_SetCalibrationResult(
    IMU_Task_CalibrationResult result)
{
    g_calibrationResult = result;
    g_reportPending     = true;
}

static void IMU_Task_ResetIntegration(uint32_t nowMs)
{
    g_sample.angleDeg           = 0.0f;
    g_sample.normalizedAngleDeg = 0.0f;
    g_previousRateDps           = 0.0f;
    g_previousRateValid         = false;
    g_lastSampleMs              = nowMs;
    g_lastSampleUs              = BSP_GetTickUs();
    IMU_Task_ResetStaticHold(nowMs);
}

static void IMU_Task_EnterReady(uint32_t nowMs)
{
    g_state        = IMU_TASK_STATE_READY;
    g_sample.state = g_state;
    g_lastSampleMs = nowMs;
}

static void IMU_Task_SetError(XV7021_Status status)
{
    g_sensorStatus      = status;
    g_state             = IMU_TASK_STATE_ERROR;
    g_sample.state      = g_state;
    g_previousRateValid = false;
    g_rotationPhase     = IMU_TASK_ROTATION_PHASE_IDLE;
    IMU_Task_ResetStaticHold(BSP_GetTickMs());
    IMU_Task_SetCalibrationResult(IMU_TASK_CAL_RESULT_SENSOR_ERROR);
    (void)HostLink_SendIMUError((int32_t)status);
}

static bool IMU_Task_ConfigureSensor(void)
{
    g_sensorStatus = XV7021_Init();
    if (g_sensorStatus != XV7021_STATUS_OK) {
        return false;
    }

    g_sensorStatus = XV7021_SetAngularRateDataFormat(
        XV7021_RATE_DATA_FORMAT_24BIT);
    if (g_sensorStatus != XV7021_STATUS_OK) {
        return false;
    }

    g_sensorStatus = XV7021_SetLowPassFilter(
        XV7021_LPF_ORDER_4, XV7021_LPF_100_HZ);
    if (g_sensorStatus != XV7021_STATUS_OK) {
        return false;
    }

    g_sensorStatus = XV7021_SetHighPassFilter(false, XV7021_HPF_0_1_HZ);
    if (g_sensorStatus != XV7021_STATUS_OK) {
        return false;
    }

    return true;
}

static void IMU_Task_InitializeSensor(uint32_t nowMs)
{
    if (!IMU_Task_ConfigureSensor()) {
        IMU_Task_SetError(g_sensorStatus);
        g_lastInitAttemptMs = nowMs;
        return;
    }

    g_sensorStatus = XV7021_STATUS_OK;
    IMU_Task_EnterReady(nowMs);
    IMU_Task_ResetIntegration(nowMs);
}

static void IMU_Task_StartZeroCalibrationAt(
    uint32_t nowMs, uint32_t durationMs)
{
    if (durationMs == 0U) {
        durationMs = IMU_TASK_DEFAULT_ZERO_CAL_MS;
    }
    if (durationMs < IMU_TASK_MIN_ZERO_CAL_MS) {
        durationMs = IMU_TASK_MIN_ZERO_CAL_MS;
    } else if (durationMs > IMU_TASK_MAX_ZERO_CAL_MS) {
        durationMs = IMU_TASK_MAX_ZERO_CAL_MS;
    }

    g_zeroCalibrationMs         = durationMs;
    g_calibrationStartMs        = nowMs;
    g_phaseStartMs              = nowMs;
    g_zeroCalibrationCollecting = false;
    IMU_Task_ResetStaticHold(nowMs);
    g_state        = IMU_TASK_STATE_ZERO_CALIBRATING;
    g_sample.state = g_state;
    IMU_Task_ResetStats();
    IMU_Task_SetCalibrationResult(IMU_TASK_CAL_RESULT_BUSY);
}

static void IMU_Task_FinishZeroCalibration(uint32_t nowMs)
{
    if (!IMU_Task_AreStatsStatic()) {
        g_state        = IMU_TASK_STATE_READY;
        g_sample.state = g_state;
        IMU_Task_SetCalibrationResult(
            IMU_TASK_CAL_RESULT_STATIC_UNSTABLE);
        return;
    }

    IMU_Task_SetBaseBiasRaw24(g_stats.meanRaw24);
    g_sample.biasDps =
        IMU_Task_Raw24ToDps(IMU_Task_GetEffectiveBiasRaw24());
    IMU_Task_SaveStoredCalibration(g_stats.count);
    IMU_Task_ResetIntegration(nowMs);
    g_state        = IMU_TASK_STATE_READY;
    g_sample.state = g_state;
    IMU_Task_SetCalibrationResult(IMU_TASK_CAL_RESULT_OK);
}

static void IMU_Task_ProcessZeroCalibration(
    uint32_t nowMs, int32_t rawAngularRate24, float rawDps)
{
    if (!g_zeroCalibrationCollecting) {
        if ((uint32_t)(nowMs - g_phaseStartMs) <
            IMU_TASK_ZERO_CAL_SETTLE_MS) {
            return;
        }

        g_zeroCalibrationCollecting = true;
        g_calibrationStartMs        = nowMs;
        IMU_Task_ResetStats();
    }

    IMU_Task_UpdateStats(rawAngularRate24, rawDps);

    if ((uint32_t)(nowMs - g_calibrationStartMs) >=
        g_zeroCalibrationMs) {
        IMU_Task_FinishZeroCalibration(nowMs);
    }
}

static void IMU_Task_StartRotationLeg(uint32_t nowMs, bool clockwise)
{
    g_rotationClockwise         = clockwise;
    g_rotationSeen              = false;
    g_rotationSaturated         = false;
    g_rotationAngleDeg          = 0.0f;
    g_rotationPreviousRateDps   = 0.0f;
    g_rotationPreviousRateValid = false;
    g_rotationLastUs            = BSP_GetTickUs();
    g_rotationStillStartMs      = nowMs;
    g_calibrationStartMs        = nowMs;
    g_phaseStartMs              = nowMs;
    g_rotationPhase             = IMU_TASK_ROTATION_PHASE_PRE_STATIC;
    IMU_Task_ResetStats();
    IMU_Task_SetCalibrationResult(IMU_TASK_CAL_RESULT_BUSY);
}

static void IMU_Task_StartRotationCalibrationAt(
    uint32_t nowMs, uint16_t turns, bool clockwise)
{
    g_rotationTurns                 = turns;
    g_rotationLegIndex              = 0U;
    g_rotationFirstAngleDeg         = 0.0f;
    g_rotationRegressionNumerator   = 0.0f;
    g_rotationRegressionDenominator = 0.0f;
    IMU_Task_ResetStaticHold(nowMs);
    g_state         = IMU_TASK_STATE_ROTATION_CALIBRATING;
    g_sample.state  = g_state;
    IMU_Task_StartRotationLeg(nowMs, clockwise);
}

static void IMU_Task_FailRotationCalibration(
    IMU_Task_CalibrationResult result)
{
    g_rotationPhase = IMU_TASK_ROTATION_PHASE_IDLE;
    g_state         = IMU_TASK_STATE_READY;
    g_sample.state  = g_state;
    IMU_Task_SetCalibrationResult(result);
}

static void IMU_Task_ProcessRotationPreStatic(
    uint32_t nowMs, int32_t rawAngularRate24, float rawDps)
{
    IMU_Task_UpdateStats(rawAngularRate24, rawDps);

    if ((uint32_t)(nowMs - g_phaseStartMs) <
        IMU_TASK_ROTATION_STATIC_MS) {
        return;
    }

    if (!IMU_Task_AreStatsStatic()) {
        IMU_Task_FailRotationCalibration(
            IMU_TASK_CAL_RESULT_STATIC_UNSTABLE);
        return;
    }

    IMU_Task_SetBaseBiasRaw24(g_stats.meanRaw24);
    g_sample.biasDps =
        IMU_Task_Raw24ToDps(IMU_Task_GetEffectiveBiasRaw24());
    g_rotationAngleDeg          = 0.0f;
    g_rotationPreviousRateDps   = 0.0f;
    g_rotationPreviousRateValid = false;
    g_rotationLastUs            = BSP_GetTickUs();
    g_rotationStillStartMs      = nowMs;
    g_phaseStartMs              = nowMs;
    g_rotationPhase             = IMU_TASK_ROTATION_PHASE_ROTATING;
    IMU_Task_ResetStats();
}

static void IMU_Task_FinishRotationCalibration(uint32_t nowMs)
{
    float measuredAngleDeg = g_rotationAngleDeg;
    float measuredAbsAngleDeg = IMU_Task_AbsFloat(measuredAngleDeg);
    float targetAbsAngleDeg = 360.0f * (float)g_rotationTurns;
    float targetSignedAngleDeg;
    float scaleCorrection;

    if (g_rotationSaturated || (measuredAbsAngleDeg <
                                (targetAbsAngleDeg *
                                 IMU_TASK_ROTATION_MIN_TARGET_RATIO))) {
        IMU_Task_FailRotationCalibration(
            IMU_TASK_CAL_RESULT_SCALE_OUT_OF_RANGE);
        return;
    }

    if (!IMU_Task_AreStatsStatic()) {
        IMU_Task_FailRotationCalibration(
            IMU_TASK_CAL_RESULT_STATIC_UNSTABLE);
        return;
    }

    if (g_rotationLegIndex == 0U) {
        g_rotationFirstAngleDeg = measuredAngleDeg;
    } else if ((measuredAngleDeg * g_rotationFirstAngleDeg) >= 0.0f) {
        IMU_Task_FailRotationCalibration(
            IMU_TASK_CAL_RESULT_SCALE_OUT_OF_RANGE);
        return;
    }

    targetSignedAngleDeg =
        (measuredAngleDeg >= 0.0f) ? targetAbsAngleDeg :
                                     -targetAbsAngleDeg;
    g_rotationRegressionNumerator +=
        targetSignedAngleDeg * measuredAngleDeg;
    g_rotationRegressionDenominator +=
        measuredAngleDeg * measuredAngleDeg;

    if ((uint8_t)(g_rotationLegIndex + 1U) <
        IMU_TASK_ROTATION_CALIBRATION_LEGS) {
        g_rotationLegIndex++;
        IMU_Task_StartRotationLeg(nowMs, !g_rotationClockwise);
        return;
    }

    if (g_rotationRegressionDenominator <= 0.0f) {
        IMU_Task_FailRotationCalibration(
            IMU_TASK_CAL_RESULT_SCALE_OUT_OF_RANGE);
        return;
    }

    scaleCorrection = g_rotationRegressionNumerator /
                      g_rotationRegressionDenominator;
    if ((scaleCorrection < IMU_TASK_SCALE_CORRECTION_MIN) ||
        (scaleCorrection > IMU_TASK_SCALE_CORRECTION_MAX)) {
        IMU_Task_FailRotationCalibration(
            IMU_TASK_CAL_RESULT_SCALE_OUT_OF_RANGE);
        return;
    }

    g_scaleCorrection        = scaleCorrection;
    g_sample.scaleCorrection = g_scaleCorrection;
    IMU_Task_SaveStoredCalibration(g_stats.count);
    IMU_Task_ResetIntegration(nowMs);
    g_rotationPhase = IMU_TASK_ROTATION_PHASE_IDLE;
    g_state         = IMU_TASK_STATE_READY;
    g_sample.state  = g_state;
    IMU_Task_SetCalibrationResult(IMU_TASK_CAL_RESULT_OK);
}

static void IMU_Task_ProcessRotation(uint32_t nowMs, uint32_t nowUs,
                                     int32_t rawAngularRate24, float rawDps)
{
    float rateDps = IMU_Task_Raw24ToDps(
        (float)rawAngularRate24 - IMU_Task_GetEffectiveBiasRaw24());

    if ((uint32_t)(nowMs - g_calibrationStartMs) >=
        IMU_TASK_ROTATION_TIMEOUT_MS) {
        IMU_Task_FailRotationCalibration(IMU_TASK_CAL_RESULT_TIMEOUT);
        return;
    }

    if (IMU_Task_AbsFloat(rawDps) >= IMU_TASK_SATURATION_RATE_DPS) {
        g_rotationSaturated = true;
    }

    if (g_rotationPreviousRateValid) {
        float dtS = (float)(uint32_t)(nowUs - g_rotationLastUs) *
                    0.000001f;

        if (dtS > 0.0f) {
            g_rotationAngleDeg += 0.5f *
                                  (g_rotationPreviousRateDps + rateDps) * dtS;
        }
    }
    g_rotationLastUs            = nowUs;
    g_rotationPreviousRateDps   = rateDps;
    g_rotationPreviousRateValid = true;

    if (IMU_Task_AbsFloat(rateDps) >= IMU_TASK_ROTATION_START_RATE_DPS) {
        g_rotationSeen         = true;
        g_rotationStillStartMs = nowMs;
        IMU_Task_ResetStats();
        return;
    }

    if (!g_rotationSeen) {
        return;
    }

    if (IMU_Task_AbsFloat(rateDps) > IMU_TASK_ROTATION_STILL_RATE_DPS) {
        g_rotationStillStartMs = nowMs;
        IMU_Task_ResetStats();
        return;
    }

    IMU_Task_UpdateStats(rawAngularRate24, rawDps);
    if ((uint32_t)(nowMs - g_rotationStillStartMs) >=
        IMU_TASK_ROTATION_STATIC_MS) {
        IMU_Task_FinishRotationCalibration(nowMs);
    }
}

static void IMU_Task_ProcessRotationCalibration(uint32_t nowMs,
                                                uint32_t nowUs, int32_t rawAngularRate24, float rawDps)
{
    if (g_rotationPhase == IMU_TASK_ROTATION_PHASE_PRE_STATIC) {
        IMU_Task_ProcessRotationPreStatic(
            nowMs, rawAngularRate24, rawDps);
    } else if (g_rotationPhase == IMU_TASK_ROTATION_PHASE_ROTATING) {
        IMU_Task_ProcessRotation(
            nowMs, nowUs, rawAngularRate24, rawDps);
    }
}

static void IMU_Task_UpdateIntegratedAngle(uint32_t nowUs, float rateDps)
{
    uint32_t dtUs;
    float dtS;

    if (!g_previousRateValid) {
        g_previousRateDps   = rateDps;
        g_previousRateValid = true;
        g_lastSampleUs      = nowUs;
        return;
    }

    dtUs = (uint32_t)(nowUs - g_lastSampleUs);
    if (dtUs == 0U) {
        return;
    }

    dtS = (float)dtUs * 0.000001f;
    g_sample.angleDeg += 0.5f * (g_previousRateDps + rateDps) * dtS;
    g_sample.normalizedAngleDeg =
        IMU_Task_NormalizeAngleDeg(g_sample.angleDeg);
    g_lastSampleUs    = nowUs;
    g_previousRateDps = rateDps;
}

static void IMU_Task_UpdateTemperature(uint32_t nowMs)
{
    int16_t rawTemperature;

    if ((uint32_t)(nowMs - g_lastTemperatureMs) <
        IMU_TASK_TEMPERATURE_PERIOD_MS) {
        return;
    }
    g_lastTemperatureMs = nowMs;

    g_sensorStatus = XV7021_ReadTemperatureRaw(&rawTemperature);
    if (g_sensorStatus == XV7021_STATUS_OK) {
        g_sample.rawTemperature = rawTemperature;
        g_sample.temperatureC =
            (float)rawTemperature / XV7021_TEMPERATURE_LSB_PER_C;
    }
}

static void IMU_Task_ReadSample(uint32_t nowMs)
{
    int32_t rawAngularRate24;
    uint32_t sampleUs;
    float rawDps;
    float correctedRateDps;
    float rateForIntegralDps;

    g_sensorStatus = XV7021_ReadAngularRateRaw24(&rawAngularRate24);
    if (g_sensorStatus != XV7021_STATUS_OK) {
        IMU_Task_SetError(g_sensorStatus);
        return;
    }
    sampleUs = BSP_GetTickUs();

    rawDps           = IMU_Task_Raw24ToDps((float)rawAngularRate24);
    correctedRateDps = IMU_Task_Raw24ToDps(
                           (float)rawAngularRate24 -
                           IMU_Task_GetEffectiveBiasRaw24()) *
                       g_scaleCorrection;
    rateForIntegralDps = correctedRateDps;
#if APP_ENABLE_IMU_STATIC_HOLD
    if (g_state == IMU_TASK_STATE_READY) {
        rateForIntegralDps =
            IMU_Task_ApplyStaticHold(nowMs, correctedRateDps);
    } else {
        IMU_Task_ResetStaticHold(nowMs);
    }
#endif

    IMU_Task_UpdateIntegratedAngle(sampleUs, rateForIntegralDps);
    g_lastSampleMs = nowMs;

    g_sample.timeMs = nowMs;
    g_sample.sampleCount++;
    g_sample.rawAngularRate24 = rawAngularRate24;
    g_sample.angularRateDps   = rateForIntegralDps;
    g_sample.biasDps =
        IMU_Task_Raw24ToDps(IMU_Task_GetEffectiveBiasRaw24());
    g_sample.scaleCorrection  = g_scaleCorrection;
    g_sample.state            = g_state;

    if (g_state == IMU_TASK_STATE_ZERO_CALIBRATING) {
        IMU_Task_ProcessZeroCalibration(
            nowMs, rawAngularRate24, rawDps);
    } else if (g_state == IMU_TASK_STATE_ROTATION_CALIBRATING) {
        IMU_Task_ProcessRotationCalibration(
            nowMs, sampleUs, rawAngularRate24, rawDps);
    }

    IMU_Task_UpdateTemperature(nowMs);
}

static void IMU_Task_ReportIfDue(uint32_t nowMs)
{
    HostLink_IMUSample hostSample;

    if (g_reportPending) {
        if (HostLink_SendIMUCalibrationState((uint8_t)g_state,
                                             (int32_t)g_calibrationResult,
                                             IMU_Task_RoundScaled(g_rotationAngleDeg,
                                                                  IMU_TASK_MILLI_SCALE),
                                             IMU_Task_RoundScaled(g_scaleCorrection,
                                                                  IMU_TASK_PPM_SCALE))) {
            g_reportPending = false;
            g_lastReportMs  = nowMs;
        }
        return;
    }

    if ((uint32_t)(nowMs - g_lastReportMs) <
        IMU_TASK_REPORT_PERIOD_MS) {
        return;
    }

    if (g_sample.sampleCount == 0U) {
        return;
    }

    hostSample.timeMs           = g_sample.timeMs;
    hostSample.state            = (uint8_t)g_state;
    hostSample.rawAngularRate24 = g_sample.rawAngularRate24;
    hostSample.angularRateMilliDps =
        IMU_Task_RoundScaled(g_sample.angularRateDps, IMU_TASK_MILLI_SCALE);
    hostSample.angleMilliDeg =
        IMU_Task_RoundScaled(g_sample.angleDeg, IMU_TASK_MILLI_SCALE);
    hostSample.biasMilliDps =
        IMU_Task_RoundScaled(g_sample.biasDps, IMU_TASK_MILLI_SCALE);
    hostSample.scalePpm =
        IMU_Task_RoundScaled(g_sample.scaleCorrection, IMU_TASK_PPM_SCALE);
    hostSample.sampleCount = g_sample.sampleCount;

    if (HostLink_SendIMUSample(&hostSample)) {
        g_lastReportMs = nowMs;
    }
}

void IMU_Task_Init(void)
{
    uint32_t nowMs = BSP_GetTickMs();

    g_state             = IMU_TASK_STATE_NOT_INITIALIZED;
    g_calibrationResult = IMU_TASK_CAL_RESULT_NONE;
    g_sensorStatus      = XV7021_STATUS_NOT_INITIALIZED;
    g_lastSampleMs      = nowMs;
    g_lastSampleUs      = BSP_GetTickUs();
    g_lastInitAttemptMs = nowMs;
    g_lastTemperatureMs = nowMs;
    g_lastReportMs      = nowMs;
    g_baseBiasRaw24     = 0.0f;
    g_adaptiveBiasDeltaRaw24 = 0.0f;
    g_scaleCorrection   = 1.0f;
    IMU_Task_LoadStoredCalibration();
    g_previousRateDps           = 0.0f;
    g_previousRateValid         = false;
    g_rotationPhase             = IMU_TASK_ROTATION_PHASE_IDLE;
    g_rotationLegIndex          = 0U;
    g_rotationFirstAngleDeg     = 0.0f;
    g_rotationRegressionNumerator = 0.0f;
    g_rotationRegressionDenominator = 0.0f;
    g_zeroCalibrationCollecting = false;
    g_reportPending             = true;
    g_sample.timeMs             = nowMs;
    g_sample.sampleCount        = 0U;
    g_sample.rawAngularRate24   = 0;
    g_sample.rawTemperature     = 0;
    g_sample.angularRateDps     = 0.0f;
    g_sample.angleDeg           = 0.0f;
    g_sample.normalizedAngleDeg = 0.0f;
    g_sample.biasDps =
        IMU_Task_Raw24ToDps(IMU_Task_GetEffectiveBiasRaw24());
    g_sample.scaleCorrection    = g_scaleCorrection;
    g_sample.temperatureC       = 0.0f;
    g_sample.state              = g_state;
    IMU_Task_ResetStats();

    IMU_Task_InitializeSensor(nowMs);
}

void IMU_Task_Run(void)
{
    uint32_t nowMs = BSP_GetTickMs();
    uint32_t nowUs = BSP_GetTickUs();

    if (g_state == IMU_TASK_STATE_ERROR) {
        if ((uint32_t)(nowMs - g_lastInitAttemptMs) >=
            IMU_TASK_RETRY_PERIOD_MS) {
            IMU_Task_InitializeSensor(nowMs);
        }
        IMU_Task_ReportIfDue(nowMs);
        return;
    }

    if ((g_state == IMU_TASK_STATE_NOT_INITIALIZED) ||
        (g_sensorStatus != XV7021_STATUS_OK)) {
        if ((uint32_t)(nowMs - g_lastInitAttemptMs) >=
            IMU_TASK_RETRY_PERIOD_MS) {
            IMU_Task_InitializeSensor(nowMs);
        }
        IMU_Task_ReportIfDue(nowMs);
        return;
    }

    if ((uint32_t)(nowUs - g_lastSampleUs) >=
        IMU_TASK_SAMPLE_PERIOD_US) {
        IMU_Task_ReadSample(nowMs);
    }

    IMU_Task_ReportIfDue(nowMs);
}

bool IMU_Task_GetSample(IMU_Task_Sample *sample)
{
    if ((sample == NULL) || (g_sample.sampleCount == 0U)) {
        return false;
    }

    *sample = g_sample;
    return true;
}

IMU_Task_State IMU_Task_GetState(void)
{
    return g_state;
}

XV7021_Status IMU_Task_GetSensorStatus(void)
{
    return g_sensorStatus;
}

IMU_Task_CalibrationResult IMU_Task_GetCalibrationResult(void)
{
    return g_calibrationResult;
}

bool IMU_Task_StartZeroCalibration(uint32_t durationMs)
{
    uint32_t nowMs = BSP_GetTickMs();

    if (g_sensorStatus != XV7021_STATUS_OK) {
        IMU_Task_SetCalibrationResult(
            IMU_TASK_CAL_RESULT_SENSOR_ERROR);
        return false;
    }

    if (IMU_Task_IsCalibrationBusy()) {
        return false;
    }

    IMU_Task_StartZeroCalibrationAt(nowMs, durationMs);
    return true;
}

bool IMU_Task_RunHardwareZeroCalibration(void)
{
    XV7021_Status status;

    if ((g_sensorStatus != XV7021_STATUS_OK) || IMU_Task_IsCalibrationBusy()) {
        return false;
    }

    status = XV7021_CalibrateZeroRate();
    if (status != XV7021_STATUS_OK) {
        IMU_Task_SetError(status);
        return false;
    }

    IMU_Task_ResetAdaptiveBias();
    g_sample.biasDps =
        IMU_Task_Raw24ToDps(IMU_Task_GetEffectiveBiasRaw24());
    IMU_Task_ResetIntegration(BSP_GetTickMs());
    return true;
}

bool IMU_Task_StartRotationCalibration(uint16_t turns, bool clockwise)
{
    uint32_t nowMs = BSP_GetTickMs();

    if (turns == 0U) {
        IMU_Task_SetCalibrationResult(
            IMU_TASK_CAL_RESULT_INVALID_ARGUMENT);
        return false;
    }

    if (g_sensorStatus != XV7021_STATUS_OK) {
        IMU_Task_SetCalibrationResult(
            IMU_TASK_CAL_RESULT_SENSOR_ERROR);
        return false;
    }

    if (IMU_Task_IsCalibrationBusy()) {
        return false;
    }

    IMU_Task_StartRotationCalibrationAt(nowMs, turns, clockwise);
    return true;
}

bool IMU_Task_ResetAngle(void)
{
    if ((g_sensorStatus != XV7021_STATUS_OK) || IMU_Task_IsCalibrationBusy()) {
        return false;
    }

    IMU_Task_ResetIntegration(BSP_GetTickMs());
    return true;
}

bool IMU_Task_IsCalibrationBusy(void)
{
    return (g_state == IMU_TASK_STATE_ZERO_CALIBRATING) ||
           (g_state == IMU_TASK_STATE_ROTATION_CALIBRATING);
}

bool IMU_Task_IsStaticBiasLearning(void)
{
#if APP_ENABLE_IMU_STATIC_HOLD
    uint32_t nowMs = BSP_GetTickMs();

    return (g_state == IMU_TASK_STATE_READY) &&
           g_staticHold.holding && g_staticHold.biasAdaptActive &&
           ((uint32_t)(nowMs - g_staticHold.biasAdaptStartMs) >=
            IMU_TASK_STATIC_BIAS_ADAPT_DELAY_MS);
#else
    return false;
#endif
}
