#ifndef DRIVERS_TMP117_H
#define DRIVERS_TMP117_H

#include <stdbool.h>
#include <stdint.h>

#define TMP117_I2C_ADDRESS        (0x48U)
#define TMP117_DEVICE_ID          (0x0117U)
#define TMP117_TEMPERATURE_LSB_C  (0.0078125f)

typedef enum {
    TMP117_STATUS_OK = 0,
    TMP117_STATUS_INVALID_ARGUMENT,
    TMP117_STATUS_NOT_FOUND,
    TMP117_STATUS_BUS_BUSY,
    TMP117_STATUS_BUS_TIMEOUT,
    TMP117_STATUS_BUS_ERROR,
    TMP117_STATUS_DEVICE_ID_MISMATCH,
} TMP117_Status;

typedef enum {
    TMP117_MODE_CONTINUOUS = 0,
    TMP117_MODE_SHUTDOWN = 1,
    TMP117_MODE_ONE_SHOT = 3,
} TMP117_ConversionMode;

typedef enum {
    TMP117_CONVERSION_CYCLE_15_5_MS = 0,
    TMP117_CONVERSION_CYCLE_125_MS,
    TMP117_CONVERSION_CYCLE_250_MS,
    TMP117_CONVERSION_CYCLE_500_MS,
    TMP117_CONVERSION_CYCLE_1_S,
    TMP117_CONVERSION_CYCLE_4_S,
    TMP117_CONVERSION_CYCLE_8_S,
    TMP117_CONVERSION_CYCLE_16_S,
} TMP117_ConversionCycle;

typedef enum {
    TMP117_AVERAGING_NONE = 0,
    TMP117_AVERAGING_8,
    TMP117_AVERAGING_32,
    TMP117_AVERAGING_64,
} TMP117_Averaging;

/* Verifies that the device ID register contains TMP117_DEVICE_ID. */
TMP117_Status TMP117_Init(void);

TMP117_Status TMP117_ReadRegister(uint8_t reg, uint16_t *value);
TMP117_Status TMP117_WriteRegister(uint8_t reg, uint16_t value);

TMP117_Status TMP117_ReadRaw(int16_t *rawTemperature);
TMP117_Status TMP117_ReadTemperatureC(float *temperatureC);
TMP117_Status TMP117_IsDataReady(bool *ready);

TMP117_Status TMP117_Configure(TMP117_ConversionMode mode,
    TMP117_ConversionCycle cycle, TMP117_Averaging averaging);
TMP117_Status TMP117_StartOneShot(void);
TMP117_Status TMP117_SoftwareReset(void);

#endif
