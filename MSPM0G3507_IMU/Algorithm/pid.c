#include "pid.h"

#include <float.h>
#include <stddef.h>

static bool PID_IsFinite(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

static float PID_Clamp(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static bool PID_ConfigIsValid(const PID_Config *config)
{
    if (config == NULL) {
        return false;
    }

    return PID_IsFinite(config->kp) && PID_IsFinite(config->ki) &&
           PID_IsFinite(config->kd) &&
           PID_IsFinite(config->samplePeriodS) &&
           (config->samplePeriodS > 0.0f) &&
           PID_IsFinite(config->outputMin) &&
           PID_IsFinite(config->outputMax) &&
           (config->outputMin <= config->outputMax);
}

bool PID_Init(PID_Controller *controller, const PID_Config *config)
{
    float discreteKi;
    float discreteKd;

    if ((controller == NULL) || !PID_ConfigIsValid(config)) {
        return false;
    }

    discreteKi = config->ki * config->samplePeriodS;
    discreteKd = config->kd / config->samplePeriodS;
    if (!PID_IsFinite(discreteKi) || !PID_IsFinite(discreteKd)) {
        return false;
    }

    controller->initialized = false;
    controller->outputMin = config->outputMin;
    controller->outputMax = config->outputMax;

    controller->cmsis.Kp = config->kp;
    controller->cmsis.Ki = discreteKi;
    controller->cmsis.Kd = discreteKd;
    arm_pid_init_f32(&controller->cmsis, 1);

    controller->initialized = true;
    return PID_Reset(controller, 0.0f);
}

bool PID_Reset(PID_Controller *controller, float initialOutput)
{
    return PID_Prime(controller, 0.0f, initialOutput);
}

bool PID_Prime(PID_Controller *controller, float currentError,
    float initialOutput)
{
    if ((controller == NULL) || !controller->initialized ||
        !PID_IsFinite(currentError) || !PID_IsFinite(initialOutput)) {
        return false;
    }

    arm_pid_reset_f32(&controller->cmsis);
    controller->cmsis.state[0] = currentError;
    controller->cmsis.state[1] = currentError;
    controller->cmsis.state[2] = PID_Clamp(initialOutput,
        controller->outputMin, controller->outputMax);
    return true;
}

bool PID_Compute(PID_Controller *controller, float setpoint,
    float measurement, float *output)
{
    float rawOutput;
    float limitedOutput;

    if (output == NULL) {
        return false;
    }
    *output = 0.0f;

    if ((controller == NULL) || !controller->initialized) {
        return false;
    }
    *output = PID_Clamp(
        0.0f, controller->outputMin, controller->outputMax);

    if (!PID_IsFinite(setpoint) || !PID_IsFinite(measurement)) {
        return false;
    }

    rawOutput = arm_pid_f32(&controller->cmsis, setpoint - measurement);
    if (!PID_IsFinite(rawOutput)) {
        (void) PID_Reset(controller, *output);
        return false;
    }

    limitedOutput = PID_Clamp(
        rawOutput, controller->outputMin, controller->outputMax);

    /* Track the limited output to prevent the incremental PID from winding up. */
    controller->cmsis.state[2] = limitedOutput;
    *output = limitedOutput;
    return true;
}
