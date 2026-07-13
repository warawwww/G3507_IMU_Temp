#include "XV7011.h"

#include <stddef.h>

#include "bsp.h"
#include "bsp_spi.h"

#define XV7011_REG_DSP_CTL1       (0x01U)
#define XV7011_REG_DSP_CTL2       (0x02U)
#define XV7011_REG_DSP_CTL3       (0x03U)
#define XV7011_REG_STATUS         (0x04U)
#define XV7011_CMD_SLEEP          (0x05U)
#define XV7011_CMD_WAKE           (0x06U)
#define XV7011_CMD_STANDBY        (0x07U)
#define XV7011_REG_TEMPERATURE    (0x08U)
#define XV7011_CMD_SOFTWARE_RESET (0x09U)
#define XV7011_REG_ANGULAR_RATE   (0x0AU)
#define XV7011_REG_OUTPUT_CTL     (0x0BU)
#define XV7011_CMD_ZERO_CALIBRATE (0x0CU)
#define XV7011_CMD_DSP_RESET      (0x0DU)
#define XV7011_CMD_MEMORY_LOAD    (0x1BU)
#define XV7011_REG_TEMP_FORMAT    (0x1CU)
#define XV7011_REG_INTERFACE_CTL  (0x1FU)

#define XV7011_SPI_READ_BIT (0x80U)

#define XV7011_INTERFACE_4WIRE_SPI (0x00U)
#define XV7011_OUTPUT_16BIT_RATE   (0x01U)
#define XV7011_TEMPERATURE_12BIT   (0x4BU)

#define XV7011_DSP_CTL1_FIXED_ONE (1U << 0)
#define XV7011_DSP_CTL1_HPF_ENABLE (1U << 1)
#define XV7011_DSP_CTL1_HPF_SHIFT  (4U)
#define XV7011_DSP_CTL1_HPF_MASK   (7U << XV7011_DSP_CTL1_HPF_SHIFT)
#define XV7011_DSP_CTL1_INVALID    ((1U << 7) | (1U << 3) | (1U << 2))

#define XV7011_DSP_CTL2_LPF_ORDER_SHIFT (4U)
#define XV7011_DSP_CTL2_LPF_ORDER_MASK  \
    (3U << XV7011_DSP_CTL2_LPF_ORDER_SHIFT)
#define XV7011_DSP_CTL2_LPF_MASK (0x0FU)

#define XV7011_DSP_CTL3_CALIBRATION_ENABLE (1U << 6)

#define XV7011_SERIAL_WAIT_MS  (1U)
#define XV7011_STARTUP_WAIT_MS (200U)

static XV7011_Status XV7011_MapSPIStatus(BSP_SPI_Status status)
{
    switch (status) {
        case BSP_SPI_STATUS_OK:
            return XV7011_STATUS_OK;

        case BSP_SPI_STATUS_INVALID_ARGUMENT:
            return XV7011_STATUS_INVALID_ARGUMENT;

        case BSP_SPI_STATUS_BUSY:
            return XV7011_STATUS_BUS_BUSY;

        case BSP_SPI_STATUS_TIMEOUT:
            return XV7011_STATUS_BUS_TIMEOUT;

        case BSP_SPI_STATUS_TRANSFER_ERROR:
        default:
            return XV7011_STATUS_BUS_ERROR;
    }
}

static void XV7011_DelayMs(uint32_t delayMs)
{
    uint32_t startMs = BSP_GetTickMs();

    while ((uint32_t) (BSP_GetTickMs() - startMs) < delayMs) {
    }
}

static bool XV7011_IsWritableRegister(uint8_t reg)
{
    return (reg == XV7011_REG_DSP_CTL1) || (reg == XV7011_REG_DSP_CTL2) ||
           (reg == XV7011_REG_DSP_CTL3) ||
           (reg == XV7011_REG_OUTPUT_CTL) ||
           (reg == XV7011_REG_TEMP_FORMAT) ||
           (reg == XV7011_REG_INTERFACE_CTL);
}

static bool XV7011_IsCommand(uint8_t command)
{
    return (command == XV7011_CMD_SLEEP) ||
           (command == XV7011_CMD_WAKE) ||
           (command == XV7011_CMD_STANDBY) ||
           (command == XV7011_CMD_SOFTWARE_RESET) ||
           (command == XV7011_CMD_ZERO_CALIBRATE) ||
           (command == XV7011_CMD_DSP_RESET) ||
           (command == XV7011_CMD_MEMORY_LOAD);
}

static XV7011_Status XV7011_SendCommand(uint8_t command)
{
    if (!XV7011_IsCommand(command)) {
        return XV7011_STATUS_INVALID_ARGUMENT;
    }

    return XV7011_MapSPIStatus(
        BSP_SPI_GyroTransfer(&command, NULL, 1U));
}

XV7011_Status XV7011_ReadRegister(uint8_t reg, uint8_t *value)
{
    uint8_t txData[2] = {(uint8_t) (XV7011_SPI_READ_BIT | reg), 0U};
    uint8_t rxData[2];
    BSP_SPI_Status busStatus;

    if ((value == NULL) || (reg > XV7011_REG_INTERFACE_CTL)) {
        return XV7011_STATUS_INVALID_ARGUMENT;
    }

    busStatus = BSP_SPI_GyroTransfer(txData, rxData, sizeof(txData));
    if (busStatus != BSP_SPI_STATUS_OK) {
        return XV7011_MapSPIStatus(busStatus);
    }

    *value = rxData[1];
    return XV7011_STATUS_OK;
}

XV7011_Status XV7011_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t txData[2] = {reg, value};

    if (!XV7011_IsWritableRegister(reg)) {
        return XV7011_STATUS_INVALID_ARGUMENT;
    }

    return XV7011_MapSPIStatus(
        BSP_SPI_GyroTransfer(txData, NULL, sizeof(txData)));
}

XV7011_Status XV7011_Init(void)
{
    uint8_t value;
    uint32_t initStartMs = BSP_GetTickMs();
    XV7011_Status status;

    XV7011_DelayMs(XV7011_SERIAL_WAIT_MS);

    status = XV7011_ReadRegister(XV7011_REG_INTERFACE_CTL, &value);
    if (status != XV7011_STATUS_OK) {
        return status;
    }

    /* Reserved bits and SPISel must be zero. I2C_EN may be 0 or its default 1. */
    if ((value & 0xFEU) != 0U) {
        return XV7011_STATUS_NOT_FOUND;
    }

    status = XV7011_WriteRegister(
        XV7011_REG_INTERFACE_CTL, XV7011_INTERFACE_4WIRE_SPI);
    if (status != XV7011_STATUS_OK) {
        return status;
    }

    status = XV7011_ReadRegister(XV7011_REG_INTERFACE_CTL, &value);
    if (status != XV7011_STATUS_OK) {
        return status;
    }
    if (value != XV7011_INTERFACE_4WIRE_SPI) {
        return XV7011_STATUS_CONFIGURATION_ERROR;
    }

    /* DspCtl1 has fixed reserved bits that make a floating MISO detectable. */
    status = XV7011_ReadRegister(XV7011_REG_DSP_CTL1, &value);
    if (status != XV7011_STATUS_OK) {
        return status;
    }
    if (((value & XV7011_DSP_CTL1_FIXED_ONE) == 0U) ||
        ((value & XV7011_DSP_CTL1_INVALID) != 0U)) {
        return XV7011_STATUS_NOT_FOUND;
    }

    status = XV7011_WriteRegister(
        XV7011_REG_OUTPUT_CTL, XV7011_OUTPUT_16BIT_RATE);
    if (status != XV7011_STATUS_OK) {
        return status;
    }

    status = XV7011_WriteRegister(
        XV7011_REG_TEMP_FORMAT, XV7011_TEMPERATURE_12BIT);
    if (status != XV7011_STATUS_OK) {
        return status;
    }

    status = XV7011_ReadRegister(XV7011_REG_OUTPUT_CTL, &value);
    if (status != XV7011_STATUS_OK) {
        return status;
    }
    if (value != XV7011_OUTPUT_16BIT_RATE) {
        return XV7011_STATUS_CONFIGURATION_ERROR;
    }

    status = XV7011_ReadRegister(XV7011_REG_TEMP_FORMAT, &value);
    if (status != XV7011_STATUS_OK) {
        return status;
    }
    if (value != XV7011_TEMPERATURE_12BIT) {
        return XV7011_STATUS_CONFIGURATION_ERROR;
    }

    while ((uint32_t) (BSP_GetTickMs() - initStartMs) <
           XV7011_STARTUP_WAIT_MS) {
    }

    return XV7011_STATUS_OK;
}

XV7011_Status XV7011_ReadStatus(uint8_t *status)
{
    return XV7011_ReadRegister(XV7011_REG_STATUS, status);
}

XV7011_Status XV7011_ReadAngularRateRaw(int16_t *rawAngularRate)
{
    uint8_t txData[3] = {
        (uint8_t) (XV7011_SPI_READ_BIT | XV7011_REG_ANGULAR_RATE), 0U, 0U};
    uint8_t rxData[3];
    BSP_SPI_Status busStatus;

    if (rawAngularRate == NULL) {
        return XV7011_STATUS_INVALID_ARGUMENT;
    }

    busStatus = BSP_SPI_GyroTransfer(txData, rxData, sizeof(txData));
    if (busStatus != BSP_SPI_STATUS_OK) {
        return XV7011_MapSPIStatus(busStatus);
    }

    *rawAngularRate =
        (int16_t) (((uint16_t) rxData[1] << 8) | rxData[2]);
    return XV7011_STATUS_OK;
}

XV7011_Status XV7011_ReadAngularRateDps(float *angularRateDps)
{
    int16_t rawAngularRate;
    XV7011_Status status;

    if (angularRateDps == NULL) {
        return XV7011_STATUS_INVALID_ARGUMENT;
    }

    status = XV7011_ReadAngularRateRaw(&rawAngularRate);
    if (status != XV7011_STATUS_OK) {
        return status;
    }

    *angularRateDps =
        (float) rawAngularRate / XV7011_ANGULAR_RATE_LSB_PER_DPS;
    return XV7011_STATUS_OK;
}

XV7011_Status XV7011_ReadTemperatureRaw(int16_t *rawTemperature)
{
    uint8_t txData[3] = {
        (uint8_t) (XV7011_SPI_READ_BIT | XV7011_REG_TEMPERATURE), 0U, 0U};
    uint8_t rxData[3];
    uint16_t raw;
    BSP_SPI_Status busStatus;

    if (rawTemperature == NULL) {
        return XV7011_STATUS_INVALID_ARGUMENT;
    }

    busStatus = BSP_SPI_GyroTransfer(txData, rxData, sizeof(txData));
    if (busStatus != BSP_SPI_STATUS_OK) {
        return XV7011_MapSPIStatus(busStatus);
    }

    raw = ((uint16_t) rxData[1] << 4) | ((uint16_t) rxData[2] >> 4);
    if ((raw & 0x0800U) != 0U) {
        raw |= 0xF000U;
    }
    *rawTemperature = (int16_t) raw;

    return XV7011_STATUS_OK;
}

XV7011_Status XV7011_ReadTemperatureC(float *temperatureC)
{
    int16_t rawTemperature;
    XV7011_Status status;

    if (temperatureC == NULL) {
        return XV7011_STATUS_INVALID_ARGUMENT;
    }

    status = XV7011_ReadTemperatureRaw(&rawTemperature);
    if (status != XV7011_STATUS_OK) {
        return status;
    }

    *temperatureC =
        (float) rawTemperature / XV7011_TEMPERATURE_LSB_PER_C;
    return XV7011_STATUS_OK;
}

XV7011_Status XV7011_SetLowPassFilter(
    XV7011_LpfOrder order, XV7011_LpfFrequency frequency)
{
    uint8_t value;
    XV7011_Status status;

    if (((uint32_t) order > XV7011_LPF_ORDER_4) ||
        ((uint32_t) frequency > XV7011_LPF_500_HZ)) {
        return XV7011_STATUS_INVALID_ARGUMENT;
    }

    status = XV7011_ReadRegister(XV7011_REG_DSP_CTL2, &value);
    if (status != XV7011_STATUS_OK) {
        return status;
    }

    value &= (uint8_t) ~(XV7011_DSP_CTL2_LPF_ORDER_MASK |
                          XV7011_DSP_CTL2_LPF_MASK);
    value |= (uint8_t) ((uint8_t) order <<
                        XV7011_DSP_CTL2_LPF_ORDER_SHIFT);
    value |= (uint8_t) frequency;

    status = XV7011_WriteRegister(XV7011_REG_DSP_CTL2, value);
    if (status != XV7011_STATUS_OK) {
        return status;
    }

    return XV7011_SendCommand(XV7011_CMD_DSP_RESET);
}

XV7011_Status XV7011_SetHighPassFilter(
    bool enable, XV7011_HpfFrequency frequency)
{
    uint8_t value;
    XV7011_Status status;

    if ((uint32_t) frequency > XV7011_HPF_10_HZ) {
        return XV7011_STATUS_INVALID_ARGUMENT;
    }

    status = XV7011_ReadRegister(XV7011_REG_DSP_CTL1, &value);
    if (status != XV7011_STATUS_OK) {
        return status;
    }

    value &= (uint8_t) ~(XV7011_DSP_CTL1_HPF_MASK |
                          XV7011_DSP_CTL1_HPF_ENABLE);
    value |=
        (uint8_t) ((uint8_t) frequency << XV7011_DSP_CTL1_HPF_SHIFT);
    if (enable) {
        value |= XV7011_DSP_CTL1_HPF_ENABLE;
    }

    status = XV7011_WriteRegister(XV7011_REG_DSP_CTL1, value);
    if (status != XV7011_STATUS_OK) {
        return status;
    }

    return XV7011_SendCommand(XV7011_CMD_DSP_RESET);
}

XV7011_Status XV7011_Sleep(void)
{
    return XV7011_SendCommand(XV7011_CMD_SLEEP);
}

XV7011_Status XV7011_Standby(void)
{
    return XV7011_SendCommand(XV7011_CMD_STANDBY);
}

XV7011_Status XV7011_Wake(void)
{
    XV7011_Status status = XV7011_SendCommand(XV7011_CMD_WAKE);

    if (status == XV7011_STATUS_OK) {
        XV7011_DelayMs(XV7011_STARTUP_WAIT_MS);
    }

    return status;
}

XV7011_Status XV7011_SoftwareReset(void)
{
    XV7011_Status status = XV7011_SendCommand(XV7011_CMD_SOFTWARE_RESET);

    if (status != XV7011_STATUS_OK) {
        return status;
    }

    return XV7011_Init();
}

XV7011_Status XV7011_CalibrateZeroRate(void)
{
    uint8_t value;
    XV7011_Status status =
        XV7011_ReadRegister(XV7011_REG_DSP_CTL3, &value);

    if (status != XV7011_STATUS_OK) {
        return status;
    }

    status = XV7011_WriteRegister(XV7011_REG_DSP_CTL3,
        value | XV7011_DSP_CTL3_CALIBRATION_ENABLE);
    if (status != XV7011_STATUS_OK) {
        return status;
    }

    return XV7011_SendCommand(XV7011_CMD_ZERO_CALIBRATE);
}
