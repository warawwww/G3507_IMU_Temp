#include "XV7021.h"

#include <stddef.h>

#include "bsp.h"
#include "bsp_spi.h"

#define XV7021_REG_DSP_CTL1       (0x01U)
#define XV7021_REG_DSP_CTL2       (0x02U)
#define XV7021_REG_DSP_CTL3       (0x03U)
#define XV7021_REG_STATUS         (0x04U)
#define XV7021_CMD_SLEEP          (0x05U)
#define XV7021_CMD_WAKE           (0x06U)
#define XV7021_CMD_STANDBY        (0x07U)
#define XV7021_REG_TEMPERATURE    (0x08U)
#define XV7021_CMD_SOFTWARE_RESET (0x09U)
#define XV7021_REG_ANGULAR_RATE   (0x0AU)
#define XV7021_REG_OUTPUT_CTL     (0x0BU)
#define XV7021_CMD_ZERO_CALIBRATE (0x0CU)
#define XV7021_REG_TEMP_FORMAT    (0x1CU)
#define XV7021_REG_INTERFACE_CTL  (0x1FU)

#define XV7021_SPI_READ_BIT (0x80U)
#define XV7021_SPI_ADDRESS_BITS_SHIFT (5U)
#define XV7021_SPI_REGISTER_MASK      (0x1FU)
#define XV7021_SPI_ADDRESS_BITS       (3U)

#define XV7021_INTERFACE_4WIRE_SPI (0x00U)
#define XV7021_OUTPUT_16BIT_RATE   (0x01U)
#define XV7021_OUTPUT_24BIT_RATE   (0x05U)
#define XV7021_TEMPERATURE_12BIT   (0x4BU)

#define XV7021_DSP_CTL1_FIXED_ONE (1U << 0)
#define XV7021_DSP_CTL1_HPF_ENABLE (1U << 1)
#define XV7021_DSP_CTL1_HPF_SHIFT  (4U)
#define XV7021_DSP_CTL1_HPF_MASK   (7U << XV7021_DSP_CTL1_HPF_SHIFT)
#define XV7021_DSP_CTL1_INVALID    ((1U << 7) | (1U << 3) | (1U << 2))

#define XV7021_DSP_CTL2_LPF_ORDER_SHIFT (4U)
#define XV7021_DSP_CTL2_LPF_ORDER_MASK  \
    (3U << XV7021_DSP_CTL2_LPF_ORDER_SHIFT)
#define XV7021_DSP_CTL2_LPF_MASK (0x0FU)

#define XV7021_DSP_CTL3_CALIBRATION_ENABLE (1U << 6)

#define XV7021_SERIAL_WAIT_MS  (1U)
#define XV7021_STARTUP_WAIT_MS (200U)

static uint8_t XV7021_BuildAddress(uint8_t reg, bool read)
{
    uint8_t address = (uint8_t) (reg & XV7021_SPI_REGISTER_MASK);

    address |= (uint8_t) ((XV7021_SPI_ADDRESS_BITS & 0x03U)
                          << XV7021_SPI_ADDRESS_BITS_SHIFT);
    if (read) {
        address |= XV7021_SPI_READ_BIT;
    }

    return address;
}

static bool XV7021_IsInterfaceValueValid(uint8_t value)
{
    return (value & 0xFEU) == 0U;
}

static XV7021_Status XV7021_MapSPIStatus(BSP_SPI_Status status)
{
    switch (status) {
        case BSP_SPI_STATUS_OK:
            return XV7021_STATUS_OK;

        case BSP_SPI_STATUS_INVALID_ARGUMENT:
            return XV7021_STATUS_INVALID_ARGUMENT;

        case BSP_SPI_STATUS_BUSY:
            return XV7021_STATUS_BUS_BUSY;

        case BSP_SPI_STATUS_TIMEOUT:
            return XV7021_STATUS_BUS_TIMEOUT;

        case BSP_SPI_STATUS_TRANSFER_ERROR:
        default:
            return XV7021_STATUS_BUS_ERROR;
    }
}

static void XV7021_DelayMs(uint32_t delayMs)
{
    uint32_t startMs = BSP_GetTickMs();

    while ((uint32_t) (BSP_GetTickMs() - startMs) < delayMs) {
    }
}

static bool XV7021_IsWritableRegister(uint8_t reg)
{
    return (reg == XV7021_REG_DSP_CTL1) || (reg == XV7021_REG_DSP_CTL2) ||
           (reg == XV7021_REG_DSP_CTL3) ||
           (reg == XV7021_REG_OUTPUT_CTL) ||
           (reg == XV7021_REG_TEMP_FORMAT) ||
           (reg == XV7021_REG_INTERFACE_CTL);
}

static bool XV7021_IsCommand(uint8_t command)
{
    return (command == XV7021_CMD_SLEEP) ||
           (command == XV7021_CMD_WAKE) ||
           (command == XV7021_CMD_STANDBY) ||
           (command == XV7021_CMD_SOFTWARE_RESET) ||
           (command == XV7021_CMD_ZERO_CALIBRATE);
}

static XV7021_Status XV7021_SendCommand(uint8_t command)
{
    uint8_t address;

    if (!XV7021_IsCommand(command)) {
        return XV7021_STATUS_INVALID_ARGUMENT;
    }

    address = XV7021_BuildAddress(command, false);
    return XV7021_MapSPIStatus(BSP_SPI_GyroTransfer(&address, NULL, 1U));
}

XV7021_Status XV7021_ReadRegister(uint8_t reg, uint8_t *value)
{
    uint8_t txData[2] = {XV7021_BuildAddress(reg, true), 0U};
    uint8_t rxData[2];
    BSP_SPI_Status busStatus;

    if ((value == NULL) || (reg > XV7021_REG_INTERFACE_CTL)) {
        return XV7021_STATUS_INVALID_ARGUMENT;
    }

    busStatus = BSP_SPI_GyroTransfer(txData, rxData, sizeof(txData));
    if (busStatus != BSP_SPI_STATUS_OK) {
        return XV7021_MapSPIStatus(busStatus);
    }

    *value = rxData[1];
    return XV7021_STATUS_OK;
}

XV7021_Status XV7021_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t txData[2] = {XV7021_BuildAddress(reg, false), value};

    if (!XV7021_IsWritableRegister(reg)) {
        return XV7021_STATUS_INVALID_ARGUMENT;
    }

    return XV7021_MapSPIStatus(
        BSP_SPI_GyroTransfer(txData, NULL, sizeof(txData)));
}

uint8_t XV7021_GetSpiAddressBits(void)
{
    return XV7021_SPI_ADDRESS_BITS;
}

XV7021_Status XV7021_SetAngularRateDataFormat(
    XV7021_RateDataFormat format)
{
    uint8_t value;

    if (format == XV7021_RATE_DATA_FORMAT_16BIT) {
        value = XV7021_OUTPUT_16BIT_RATE;
    } else if (format == XV7021_RATE_DATA_FORMAT_24BIT) {
        value = XV7021_OUTPUT_24BIT_RATE;
    } else {
        return XV7021_STATUS_INVALID_ARGUMENT;
    }

    return XV7021_WriteRegister(XV7021_REG_OUTPUT_CTL, value);
}

XV7021_Status XV7021_Init(void)
{
    uint8_t value;
    uint32_t initStartMs = BSP_GetTickMs();
    XV7021_Status status;

    XV7021_DelayMs(XV7021_SERIAL_WAIT_MS);

    status = XV7021_ReadRegister(XV7021_REG_INTERFACE_CTL, &value);
    if (status != XV7021_STATUS_OK) {
        return status;
    }
    if (!XV7021_IsInterfaceValueValid(value)) {
        return XV7021_STATUS_NOT_FOUND;
    }

    status = XV7021_WriteRegister(
        XV7021_REG_INTERFACE_CTL, XV7021_INTERFACE_4WIRE_SPI);
    if (status != XV7021_STATUS_OK) {
        return status;
    }

    status = XV7021_ReadRegister(XV7021_REG_INTERFACE_CTL, &value);
    if (status != XV7021_STATUS_OK) {
        return status;
    }
    if (value != XV7021_INTERFACE_4WIRE_SPI) {
        return XV7021_STATUS_CONFIGURATION_ERROR;
    }

    /* DspCtl1 的固定保留位可用于识别 MISO 悬空或未真正读到芯片的情况。 */
    status = XV7021_ReadRegister(XV7021_REG_DSP_CTL1, &value);
    if (status != XV7021_STATUS_OK) {
        return status;
    }
    if (((value & XV7021_DSP_CTL1_FIXED_ONE) == 0U) ||
        ((value & XV7021_DSP_CTL1_INVALID) != 0U)) {
        return XV7021_STATUS_NOT_FOUND;
    }

    status = XV7021_SetAngularRateDataFormat(
        XV7021_RATE_DATA_FORMAT_24BIT);
    if (status != XV7021_STATUS_OK) {
        return status;
    }

    status = XV7021_WriteRegister(
        XV7021_REG_TEMP_FORMAT, XV7021_TEMPERATURE_12BIT);
    if (status != XV7021_STATUS_OK) {
        return status;
    }

    status = XV7021_ReadRegister(XV7021_REG_OUTPUT_CTL, &value);
    if (status != XV7021_STATUS_OK) {
        return status;
    }
    if (value != XV7021_OUTPUT_24BIT_RATE) {
        return XV7021_STATUS_CONFIGURATION_ERROR;
    }

    status = XV7021_ReadRegister(XV7021_REG_TEMP_FORMAT, &value);
    if (status != XV7021_STATUS_OK) {
        return status;
    }
    if (value != XV7021_TEMPERATURE_12BIT) {
        return XV7021_STATUS_CONFIGURATION_ERROR;
    }

    while ((uint32_t) (BSP_GetTickMs() - initStartMs) <
           XV7021_STARTUP_WAIT_MS) {
    }

    return XV7021_STATUS_OK;
}

XV7021_Status XV7021_ReadStatus(uint8_t *status)
{
    return XV7021_ReadRegister(XV7021_REG_STATUS, status);
}

XV7021_Status XV7021_ReadAngularRateRaw(int16_t *rawAngularRate)
{
    int32_t rawAngularRate24;
    XV7021_Status status;

    if (rawAngularRate == NULL) {
        return XV7021_STATUS_INVALID_ARGUMENT;
    }

    status = XV7021_ReadAngularRateRaw24(&rawAngularRate24);
    if (status != XV7021_STATUS_OK) {
        return status;
    }

    *rawAngularRate = (int16_t) (rawAngularRate24 / 256);
    return XV7021_STATUS_OK;
}

XV7021_Status XV7021_ReadAngularRateRaw24(int32_t *rawAngularRate)
{
    uint8_t txData[4] = {
        XV7021_BuildAddress(XV7021_REG_ANGULAR_RATE, true), 0U, 0U, 0U};
    uint8_t rxData[4];
    uint32_t raw;
    BSP_SPI_Status busStatus;

    if (rawAngularRate == NULL) {
        return XV7021_STATUS_INVALID_ARGUMENT;
    }

    busStatus = BSP_SPI_GyroTransfer(txData, rxData, sizeof(txData));
    if (busStatus != BSP_SPI_STATUS_OK) {
        return XV7021_MapSPIStatus(busStatus);
    }

    raw = ((uint32_t) rxData[1] << 16) |
          ((uint32_t) rxData[2] << 8) | rxData[3];
    if ((raw & 0x00800000UL) != 0UL) {
        raw |= 0xFF000000UL;
    }

    *rawAngularRate = (int32_t) raw;
    return XV7021_STATUS_OK;
}

XV7021_Status XV7021_ReadAngularRateDps(float *angularRateDps)
{
    int32_t rawAngularRate;
    XV7021_Status status;

    if (angularRateDps == NULL) {
        return XV7021_STATUS_INVALID_ARGUMENT;
    }

    status = XV7021_ReadAngularRateRaw24(&rawAngularRate);
    if (status != XV7021_STATUS_OK) {
        return status;
    }

    *angularRateDps =
        (float) rawAngularRate / XV7021_ANGULAR_RATE_24BIT_LSB_PER_DPS;
    return XV7021_STATUS_OK;
}

XV7021_Status XV7021_ReadAngularRateDps24(float *angularRateDps)
{
    int32_t rawAngularRate;
    XV7021_Status status;

    if (angularRateDps == NULL) {
        return XV7021_STATUS_INVALID_ARGUMENT;
    }

    status = XV7021_ReadAngularRateRaw24(&rawAngularRate);
    if (status != XV7021_STATUS_OK) {
        return status;
    }

    *angularRateDps =
        (float) rawAngularRate / XV7021_ANGULAR_RATE_24BIT_LSB_PER_DPS;
    return XV7021_STATUS_OK;
}

XV7021_Status XV7021_ReadTemperatureRaw(int16_t *rawTemperature)
{
    uint8_t txData[3] = {
        XV7021_BuildAddress(XV7021_REG_TEMPERATURE, true), 0U, 0U};
    uint8_t rxData[3];
    uint16_t raw;
    BSP_SPI_Status busStatus;

    if (rawTemperature == NULL) {
        return XV7021_STATUS_INVALID_ARGUMENT;
    }

    busStatus = BSP_SPI_GyroTransfer(txData, rxData, sizeof(txData));
    if (busStatus != BSP_SPI_STATUS_OK) {
        return XV7021_MapSPIStatus(busStatus);
    }

    raw = ((uint16_t) rxData[1] << 4) | ((uint16_t) rxData[2] >> 4);
    if ((raw & 0x0800U) != 0U) {
        raw |= 0xF000U;
    }
    *rawTemperature = (int16_t) raw;

    return XV7021_STATUS_OK;
}

XV7021_Status XV7021_ReadTemperatureC(float *temperatureC)
{
    int16_t rawTemperature;
    XV7021_Status status;

    if (temperatureC == NULL) {
        return XV7021_STATUS_INVALID_ARGUMENT;
    }

    status = XV7021_ReadTemperatureRaw(&rawTemperature);
    if (status != XV7021_STATUS_OK) {
        return status;
    }

    *temperatureC =
        (float) rawTemperature / XV7021_TEMPERATURE_LSB_PER_C;
    return XV7021_STATUS_OK;
}

XV7021_Status XV7021_SetLowPassFilter(
    XV7021_LpfOrder order, XV7021_LpfFrequency frequency)
{
    uint8_t value;
    XV7021_Status status;

    if (((uint32_t) order > XV7021_LPF_ORDER_4) ||
        ((uint32_t) frequency > XV7021_LPF_500_HZ)) {
        return XV7021_STATUS_INVALID_ARGUMENT;
    }

    status = XV7021_ReadRegister(XV7021_REG_DSP_CTL2, &value);
    if (status != XV7021_STATUS_OK) {
        return status;
    }

    value &= (uint8_t) ~(XV7021_DSP_CTL2_LPF_ORDER_MASK |
                          XV7021_DSP_CTL2_LPF_MASK);
    value |= (uint8_t) ((uint8_t) order <<
                        XV7021_DSP_CTL2_LPF_ORDER_SHIFT);
    value |= (uint8_t) frequency;

    status = XV7021_WriteRegister(XV7021_REG_DSP_CTL2, value);
    if (status != XV7021_STATUS_OK) {
        return status;
    }

    return XV7021_STATUS_OK;
}

XV7021_Status XV7021_SetHighPassFilter(
    bool enable, XV7021_HpfFrequency frequency)
{
    uint8_t value;
    XV7021_Status status;

    if ((uint32_t) frequency > XV7021_HPF_10_HZ) {
        return XV7021_STATUS_INVALID_ARGUMENT;
    }

    status = XV7021_ReadRegister(XV7021_REG_DSP_CTL1, &value);
    if (status != XV7021_STATUS_OK) {
        return status;
    }

    value &= (uint8_t) ~(XV7021_DSP_CTL1_HPF_MASK |
                          XV7021_DSP_CTL1_HPF_ENABLE);
    value |=
        (uint8_t) ((uint8_t) frequency << XV7021_DSP_CTL1_HPF_SHIFT);
    if (enable) {
        value |= XV7021_DSP_CTL1_HPF_ENABLE;
    }

    status = XV7021_WriteRegister(XV7021_REG_DSP_CTL1, value);
    if (status != XV7021_STATUS_OK) {
        return status;
    }

    return XV7021_STATUS_OK;
}

XV7021_Status XV7021_Sleep(void)
{
    return XV7021_SendCommand(XV7021_CMD_SLEEP);
}

XV7021_Status XV7021_Standby(void)
{
    return XV7021_SendCommand(XV7021_CMD_STANDBY);
}

XV7021_Status XV7021_Wake(void)
{
    XV7021_Status status = XV7021_SendCommand(XV7021_CMD_WAKE);

    if (status == XV7021_STATUS_OK) {
        XV7021_DelayMs(XV7021_STARTUP_WAIT_MS);
    }

    return status;
}

XV7021_Status XV7021_SoftwareReset(void)
{
    XV7021_Status status = XV7021_SendCommand(XV7021_CMD_SOFTWARE_RESET);

    if (status != XV7021_STATUS_OK) {
        return status;
    }

    return XV7021_Init();
}

XV7021_Status XV7021_CalibrateZeroRate(void)
{
    uint8_t value;
    XV7021_Status status =
        XV7021_ReadRegister(XV7021_REG_DSP_CTL3, &value);

    if (status != XV7021_STATUS_OK) {
        return status;
    }

    status = XV7021_WriteRegister(XV7021_REG_DSP_CTL3,
        value | XV7021_DSP_CTL3_CALIBRATION_ENABLE);
    if (status != XV7021_STATUS_OK) {
        return status;
    }

    return XV7021_SendCommand(XV7021_CMD_ZERO_CALIBRATE);
}
