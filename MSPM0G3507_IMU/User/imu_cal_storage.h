#ifndef USER_IMU_CAL_STORAGE_H
#define USER_IMU_CAL_STORAGE_H

#include <stdbool.h>
#include <stdint.h>

#define IMU_CAL_STORAGE_FLASH_ADDRESS (0x0001FC00UL)
#define IMU_CAL_STORAGE_FLASH_SIZE    (1024UL)

typedef struct {
    float biasRaw24;
    float scaleCorrection;
    float biasTemperatureC;
    uint32_t sampleCount;
} IMU_CalStorage_Data;

bool IMU_CalStorage_Load(IMU_CalStorage_Data *data);
bool IMU_CalStorage_Save(const IMU_CalStorage_Data *data);

#endif
