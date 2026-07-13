#ifndef DRIVERS_XV7011_H
#define DRIVERS_XV7011_H

#include <stdbool.h>
#include <stdint.h>

#define XV7011_ANGULAR_RATE_LSB_PER_DPS (280.0f)
#define XV7011_TEMPERATURE_LSB_PER_C    (16.0f)

typedef enum {
    XV7011_STATUS_NOT_INITIALIZED = 0,
    XV7011_STATUS_OK,
    XV7011_STATUS_INVALID_ARGUMENT,
    XV7011_STATUS_NOT_FOUND,
    XV7011_STATUS_BUS_BUSY,
    XV7011_STATUS_BUS_TIMEOUT,
    XV7011_STATUS_BUS_ERROR,
    XV7011_STATUS_CONFIGURATION_ERROR,
} XV7011_Status;

typedef enum {
    XV7011_LPF_ORDER_2 = 0,
    XV7011_LPF_ORDER_3,
    XV7011_LPF_ORDER_4,
} XV7011_LpfOrder;

typedef enum {
    XV7011_LPF_10_HZ = 0,
    XV7011_LPF_35_HZ,
    XV7011_LPF_45_HZ,
    XV7011_LPF_50_HZ,
    XV7011_LPF_70_HZ,
    XV7011_LPF_85_HZ,
    XV7011_LPF_100_HZ,
    XV7011_LPF_140_HZ,
    XV7011_LPF_175_HZ,
    XV7011_LPF_200_HZ,
    XV7011_LPF_285_HZ,
    XV7011_LPF_345_HZ,
    XV7011_LPF_400_HZ,
    XV7011_LPF_500_HZ,
} XV7011_LpfFrequency;

typedef enum {
    XV7011_HPF_0_01_HZ = 0,
    XV7011_HPF_0_03_HZ,
    XV7011_HPF_0_1_HZ,
    XV7011_HPF_0_3_HZ,
    XV7011_HPF_1_HZ,
    XV7011_HPF_3_HZ,
    XV7011_HPF_10_HZ,
} XV7011_HpfFrequency;

/*
 * Selects 4-wire SPI, 16-bit angular-rate output and 12-bit temperature
 * output, then waits for the datasheet's 200 ms angular-rate startup time.
 */
XV7011_Status XV7011_Init(void);

XV7011_Status XV7011_ReadRegister(uint8_t reg, uint8_t *value);
XV7011_Status XV7011_WriteRegister(uint8_t reg, uint8_t value);

XV7011_Status XV7011_ReadStatus(uint8_t *status);
XV7011_Status XV7011_ReadAngularRateRaw(int16_t *rawAngularRate);
XV7011_Status XV7011_ReadAngularRateDps(float *angularRateDps);
XV7011_Status XV7011_ReadTemperatureRaw(int16_t *rawTemperature);
XV7011_Status XV7011_ReadTemperatureC(float *temperatureC);

XV7011_Status XV7011_SetLowPassFilter(
    XV7011_LpfOrder order, XV7011_LpfFrequency frequency);
XV7011_Status XV7011_SetHighPassFilter(
    bool enable, XV7011_HpfFrequency frequency);

XV7011_Status XV7011_Sleep(void);
XV7011_Status XV7011_Standby(void);
XV7011_Status XV7011_Wake(void);
XV7011_Status XV7011_SoftwareReset(void);
XV7011_Status XV7011_CalibrateZeroRate(void);

#endif
