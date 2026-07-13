#include "bsp_spi.h"

#include <stdbool.h>

#include "bsp.h"
#include "ti_msp_dl_config.h"

#define BSP_SPI_GYRO_BIT_RATE_HZ  (1000000U)
#define BSP_SPI_TIMEOUT_MARGIN_MS (2U)

#define BSP_SPI_ERROR_MASK                                               \
    (DL_SPI_INTERRUPT_RX_OVERFLOW | DL_SPI_INTERRUPT_TX_UNDERFLOW |     \
        DL_SPI_INTERRUPT_PARITY_ERROR)

static bool g_spiGyroBusy;

static uint32_t BSP_SPI_GetTimeoutMs(size_t length)
{
    uint64_t wireTimeUs =
        (((uint64_t) length * 8U * 1000000U) + BSP_SPI_GYRO_BIT_RATE_HZ -
            1U) /
        BSP_SPI_GYRO_BIT_RATE_HZ;

    return (uint32_t) ((wireTimeUs + 999U) / 1000U) +
           BSP_SPI_TIMEOUT_MARGIN_MS;
}

static void BSP_SPI_DrainRxFIFO(void)
{
    while (!DL_SPI_isRXFIFOEmpty(SPI_GYRO_INST)) {
        (void) DL_SPI_receiveData8(SPI_GYRO_INST);
    }
}

void BSP_SPI_Init(void)
{
    g_spiGyroBusy = false;
    DL_GPIO_setPins(
        GPIO_GRP_GYRO_PORT, GPIO_GRP_GYRO_GYRO_CS_N_PIN);
    BSP_SPI_DrainRxFIFO();
    DL_SPI_clearInterruptStatus(SPI_GYRO_INST, BSP_SPI_ERROR_MASK);
}

BSP_SPI_Status BSP_SPI_GyroTransfer(
    const uint8_t *txData, uint8_t *rxData, size_t length)
{
    size_t txCount = 0U;
    size_t rxCount = 0U;
    uint32_t startMs;
    uint32_t timeoutMs;
    BSP_SPI_Status status = BSP_SPI_STATUS_OK;

    if ((length == 0U) || ((txData == NULL) && (rxData == NULL))) {
        return BSP_SPI_STATUS_INVALID_ARGUMENT;
    }

    if (g_spiGyroBusy) {
        return BSP_SPI_STATUS_BUSY;
    }

    timeoutMs = BSP_SPI_GetTimeoutMs(length);
    startMs   = BSP_GetTickMs();
    while (DL_SPI_isBusy(SPI_GYRO_INST)) {
        if ((uint32_t) (BSP_GetTickMs() - startMs) >= timeoutMs) {
            return BSP_SPI_STATUS_BUSY;
        }
    }

    g_spiGyroBusy = true;
    BSP_SPI_DrainRxFIFO();
    DL_SPI_clearInterruptStatus(SPI_GYRO_INST, BSP_SPI_ERROR_MASK);

    DL_GPIO_clearPins(
        GPIO_GRP_GYRO_PORT, GPIO_GRP_GYRO_GYRO_CS_N_PIN);
    delay_cycles(2U);

    startMs = BSP_GetTickMs();
    while (rxCount < length) {
        while ((txCount < length) &&
               !DL_SPI_isTXFIFOFull(SPI_GYRO_INST)) {
            DL_SPI_transmitData8(
                SPI_GYRO_INST, (txData != NULL) ? txData[txCount] : 0U);
            txCount++;
        }

        while ((rxCount < length) &&
               !DL_SPI_isRXFIFOEmpty(SPI_GYRO_INST)) {
            uint8_t data = DL_SPI_receiveData8(SPI_GYRO_INST);

            if (rxData != NULL) {
                rxData[rxCount] = data;
            }
            rxCount++;
        }

        if ((DL_SPI_getRawInterruptStatus(
                 SPI_GYRO_INST, BSP_SPI_ERROR_MASK) &
                BSP_SPI_ERROR_MASK) != 0U) {
            status = BSP_SPI_STATUS_TRANSFER_ERROR;
            break;
        }

        if ((uint32_t) (BSP_GetTickMs() - startMs) >= timeoutMs) {
            status = BSP_SPI_STATUS_TIMEOUT;
            break;
        }
    }

    while ((status == BSP_SPI_STATUS_OK) &&
           DL_SPI_isBusy(SPI_GYRO_INST)) {
        if ((uint32_t) (BSP_GetTickMs() - startMs) >= timeoutMs) {
            status = BSP_SPI_STATUS_TIMEOUT;
        }
    }

    delay_cycles(8U);
    DL_GPIO_setPins(
        GPIO_GRP_GYRO_PORT, GPIO_GRP_GYRO_GYRO_CS_N_PIN);
    BSP_SPI_DrainRxFIFO();
    DL_SPI_clearInterruptStatus(SPI_GYRO_INST, BSP_SPI_ERROR_MASK);
    g_spiGyroBusy = false;

    return status;
}
