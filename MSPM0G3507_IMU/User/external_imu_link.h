#ifndef USER_EXTERNAL_IMU_LINK_H
#define USER_EXTERNAL_IMU_LINK_H

#include <stdbool.h>
#include <stdint.h>

void ExternalIMULink_Init(void);
void ExternalIMULink_Run(uint8_t appState);

bool ExternalIMULink_TakeZeroCalibrationRequest(void);
bool ExternalIMULink_TakeAutoCRequest(void);
bool ExternalIMULink_TakeRotationCalibrationRequest(void);
bool ExternalIMULink_TakeAngleResetRequest(void);

#endif
