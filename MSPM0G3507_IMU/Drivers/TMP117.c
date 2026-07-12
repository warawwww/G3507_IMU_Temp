#include "TMP117.h"

#include <stddef.h>

#include "bsp.h"
#include "bsp_i2c.h"

#define TMP117_REG_TEMPERATURE   (0x00U)
#define TMP117_REG_CONFIGURATION (0x01U)
#define TMP117_REG_DEVICE_ID     (0x0FU)

#define TMP117_DEVICE_ID_MASK        (0x0FFFU)
#define TMP117_CONFIG_DATA_READY     (1U << 13)
#define TMP117_CONFIG_MODE_SHIFT     (10U)
#define TMP117_CONFIG_MODE_MASK      (3U << TMP117_CONFIG_MODE_SHIFT)
#define TMP117_CONFIG_CYCLE_SHIFT    (7U)
#define TMP117_CONFIG_CYCLE_MASK     (7U << TMP117_CONFIG_CYCLE_SHIFT)
#define TMP117_CONFIG_AVERAGE_SHIFT  (5U)
#define TMP117_CONFIG_AVERAGE_MASK   (3U << TMP117_CONFIG_AVERAGE_SHIFT)
#define TMP117_CONFIG_SOFTWARE_RESET (1U << 1)

#define TMP117_SOFTWARE_RESET_TIME_MS (2U)

static TMP117_Status TMP117_MapI2CStatus(BSP_I2C_Status status)
{
    switch (status) {
        case BSP_I2C_STATUS_OK:
            return TMP117_STATUS_OK;

        case BSP_I2C_STATUS_INVALID_ARGUMENT:
            return TMP117_STATUS_INVALID_ARGUMENT;

        case BSP_I2C_STATUS_BUSY:
            return TMP117_STATUS_BUS_BUSY;

        case BSP_I2C_STATUS_TIMEOUT:
            return TMP117_STATUS_BUS_TIMEOUT;

        case BSP_I2C_STATUS_NACK:
            return TMP117_STATUS_NOT_FOUND;

        case BSP_I2C_STATUS_ARBITRATION_LOST:
        case BSP_I2C_STATUS_TRANSFER_ERROR:
        default:
            return TMP117_STATUS_BUS_ERROR;
    }
}

TMP117_Status TMP117_ReadRegister(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    BSP_I2C_Status busStatus;

    if (value == NULL) {
        return TMP117_STATUS_INVALID_ARGUMENT;
    }

    busStatus = BSP_I2C_WriteRead(
        TMP117_I2C_ADDRESS, &reg, 1U, data, sizeof(data));
    if (busStatus != BSP_I2C_STATUS_OK) {
        return TMP117_MapI2CStatus(busStatus);
    }

    *value = ((uint16_t) data[0] << 8) | data[1];
    return TMP117_STATUS_OK;
}

TMP117_Status TMP117_WriteRegister(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {
        reg,
        (uint8_t) (value >> 8),
        (uint8_t) value,
    };

    return TMP117_MapI2CStatus(
        BSP_I2C_Write(TMP117_I2C_ADDRESS, data, sizeof(data)));
}

TMP117_Status TMP117_Init(void)
{
    uint16_t deviceId;
    TMP117_Status status = TMP117_ReadRegister(TMP117_REG_DEVICE_ID, &deviceId);

    if (status != TMP117_STATUS_OK) {
        return status;
    }

    if ((deviceId & TMP117_DEVICE_ID_MASK) != TMP117_DEVICE_ID) {
        return TMP117_STATUS_DEVICE_ID_MISMATCH;
    }

    return TMP117_STATUS_OK;
}

TMP117_Status TMP117_ReadRaw(int16_t *rawTemperature)
{
    uint16_t value;
    TMP117_Status status;

    if (rawTemperature == NULL) {
        return TMP117_STATUS_INVALID_ARGUMENT;
    }

    status = TMP117_ReadRegister(TMP117_REG_TEMPERATURE, &value);
    if (status != TMP117_STATUS_OK) {
        return status;
    }

    *rawTemperature = (int16_t) value;
    return TMP117_STATUS_OK;
}

TMP117_Status TMP117_ReadTemperatureC(float *temperatureC)
{
    int16_t rawTemperature;
    TMP117_Status status;

    if (temperatureC == NULL) {
        return TMP117_STATUS_INVALID_ARGUMENT;
    }

    status = TMP117_ReadRaw(&rawTemperature);
    if (status != TMP117_STATUS_OK) {
        return status;
    }

    *temperatureC = (float) rawTemperature * TMP117_TEMPERATURE_LSB_C;
    return TMP117_STATUS_OK;
}

TMP117_Status TMP117_IsDataReady(bool *ready)
{
    uint16_t configuration;
    TMP117_Status status;

    if (ready == NULL) {
        return TMP117_STATUS_INVALID_ARGUMENT;
    }

    status = TMP117_ReadRegister(TMP117_REG_CONFIGURATION, &configuration);
    if (status != TMP117_STATUS_OK) {
        return status;
    }

    *ready = (configuration & TMP117_CONFIG_DATA_READY) != 0U;
    return TMP117_STATUS_OK;
}

TMP117_Status TMP117_Configure(TMP117_ConversionMode mode,
    TMP117_ConversionCycle cycle, TMP117_Averaging averaging)
{
    uint16_t configuration;
    TMP117_Status status;

    if (((mode != TMP117_MODE_CONTINUOUS) &&
            (mode != TMP117_MODE_SHUTDOWN) &&
            (mode != TMP117_MODE_ONE_SHOT)) ||
        ((uint32_t) cycle > TMP117_CONVERSION_CYCLE_16_S) ||
        ((uint32_t) averaging > TMP117_AVERAGING_64)) {
        return TMP117_STATUS_INVALID_ARGUMENT;
    }

    status = TMP117_ReadRegister(TMP117_REG_CONFIGURATION, &configuration);
    if (status != TMP117_STATUS_OK) {
        return status;
    }

    configuration &= (uint16_t) ~(TMP117_CONFIG_MODE_MASK |
                                    TMP117_CONFIG_CYCLE_MASK |
                                    TMP117_CONFIG_AVERAGE_MASK);
    configuration |= (uint16_t) ((uint16_t) mode << TMP117_CONFIG_MODE_SHIFT);
    configuration |=
        (uint16_t) ((uint16_t) cycle << TMP117_CONFIG_CYCLE_SHIFT);
    configuration |=
        (uint16_t) ((uint16_t) averaging << TMP117_CONFIG_AVERAGE_SHIFT);

    return TMP117_WriteRegister(TMP117_REG_CONFIGURATION, configuration);
}

TMP117_Status TMP117_StartOneShot(void)
{
    uint16_t configuration;
    TMP117_Status status =
        TMP117_ReadRegister(TMP117_REG_CONFIGURATION, &configuration);

    if (status != TMP117_STATUS_OK) {
        return status;
    }

    configuration &= (uint16_t) ~TMP117_CONFIG_MODE_MASK;
    configuration |=
        (uint16_t) (TMP117_MODE_ONE_SHOT << TMP117_CONFIG_MODE_SHIFT);

    return TMP117_WriteRegister(TMP117_REG_CONFIGURATION, configuration);
}

TMP117_Status TMP117_SoftwareReset(void)
{
    uint16_t configuration;
    uint32_t startMs;
    TMP117_Status status =
        TMP117_ReadRegister(TMP117_REG_CONFIGURATION, &configuration);

    if (status != TMP117_STATUS_OK) {
        return status;
    }

    status = TMP117_WriteRegister(TMP117_REG_CONFIGURATION,
        configuration | TMP117_CONFIG_SOFTWARE_RESET);
    if (status != TMP117_STATUS_OK) {
        return status;
    }

    startMs = BSP_GetTickMs();
    while ((uint32_t) (BSP_GetTickMs() - startMs) <
           TMP117_SOFTWARE_RESET_TIME_MS) {
    }

    return TMP117_STATUS_OK;
}
