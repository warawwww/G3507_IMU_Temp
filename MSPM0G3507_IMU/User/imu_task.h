#ifndef USER_IMU_TASK_H
#define USER_IMU_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "XV7021.h"

typedef enum {
    IMU_TASK_STATE_NOT_INITIALIZED = 0,
    IMU_TASK_STATE_READY,
    IMU_TASK_STATE_ZERO_CALIBRATING,
    IMU_TASK_STATE_ROTATION_CALIBRATING,
    IMU_TASK_STATE_ERROR,
} IMU_Task_State;

typedef enum {
    IMU_TASK_CAL_RESULT_NONE = 0,
    IMU_TASK_CAL_RESULT_BUSY,
    IMU_TASK_CAL_RESULT_OK,
    IMU_TASK_CAL_RESULT_INVALID_ARGUMENT,
    IMU_TASK_CAL_RESULT_SENSOR_ERROR,
    IMU_TASK_CAL_RESULT_STATIC_UNSTABLE,
    IMU_TASK_CAL_RESULT_TIMEOUT,
    IMU_TASK_CAL_RESULT_SCALE_OUT_OF_RANGE,
} IMU_Task_CalibrationResult;

typedef struct {
    uint32_t timeMs;
    uint32_t sampleCount;
    int32_t rawAngularRate24;
    int16_t rawTemperature;
    float angularRateDps;
    float angleDeg;
    float normalizedAngleDeg;
    float biasDps;
    float scaleCorrection;
    float temperatureC;
    IMU_Task_State state;
} IMU_Task_Sample;

void IMU_Task_Init(void);
void IMU_Task_Run(void);

bool IMU_Task_GetSample(IMU_Task_Sample *sample);
IMU_Task_State IMU_Task_GetState(void);
XV7021_Status IMU_Task_GetSensorStatus(void);
IMU_Task_CalibrationResult IMU_Task_GetCalibrationResult(void);

bool IMU_Task_StartZeroCalibration(uint32_t durationMs);
bool IMU_Task_RunHardwareZeroCalibration(void);
bool IMU_Task_StartRotationCalibration(uint16_t turns, bool clockwise);
bool IMU_Task_ResetAngle(void);
bool IMU_Task_IsCalibrationBusy(void);

#endif
