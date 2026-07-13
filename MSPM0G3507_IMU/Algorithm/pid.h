#ifndef ALGORITHM_PID_H
#define ALGORITHM_PID_H

#include <stdbool.h>

#include "arm_math.h"

/**
 * PID gains use continuous-time units. The wrapper converts Ki and Kd to the
 * discrete gains expected by CMSIS-DSP using samplePeriodS.
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float samplePeriodS;
    float outputMin;
    float outputMax;
} PID_Config;

typedef struct {
    arm_pid_instance_f32 cmsis;
    float outputMin;
    float outputMax;
    bool initialized;
} PID_Controller;

/** Initialize a PID controller and clear its history. */
bool PID_Init(PID_Controller *controller, const PID_Config *config);

/**
 * Clear the error history and start from initialOutput.
 * initialOutput is clamped to the configured output limits.
 */
bool PID_Reset(PID_Controller *controller, float initialOutput);

/**
 * Run one PID step using error = setpoint - measurement.
 * Call this function at the fixed period configured in samplePeriodS.
 */
bool PID_Compute(PID_Controller *controller, float setpoint,
    float measurement, float *output);

#endif
