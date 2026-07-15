#include "bsp_spi.h"

#include <stdbool.h>

#include "bsp.h"
#include "ti_msp_dl_config.h"

#define BSP_SPI_GYRO_BIT_RATE_HZ  (5000000U)
#define BSP_SPI_GYRO_CLOCK_DIVIDER \
    (((CPUCLK_FREQ + ((2U * BSP_SPI_GYRO_BIT_RATE_HZ) - 1U)) / \
         (2U * BSP_SPI_GYRO_BIT_RATE_HZ)) - \
        1U)
#define BSP_SPI_TIMEOUT_MARGIN_MS (2U)
#define BSP_SPI_GYRO_RX_DMA_CHANNEL (4U)
#define BSP_SPI_GYRO_TX_DMA_CHANNEL (5U)
#define BSP_SPI_GYRO_RX_DMA_TRIGGER DMA_SPI0_RX_TRIG
#define BSP_SPI_GYRO_TX_DMA_TRIGGER DMA_SPI0_TX_TRIG

#define BSP_SPI_ERROR_MASK                                               \
    (DL_SPI_INTERRUPT_RX_OVERFLOW | DL_SPI_INTERRUPT_TX_UNDERFLOW |     \
        DL_SPI_INTERRUPT_PARITY_ERROR)

static bool g_spiGyroBusy;
static uint8_t g_spiGyroDummyTx;
static uint8_t g_spiGyroDummyRx;

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

static void BSP_SPI_ConfigureGyroDMAChannel(uint8_t channel, uint8_t trigger)
{
    DL_DMA_disableChannel(DMA, channel);
    DL_DMA_setTrigger(
        DMA, channel, trigger, DL_DMA_TRIGGER_TYPE_EXTERNAL);
    DL_DMA_configTransfer(DMA, channel, DL_DMA_SINGLE_TRANSFER_MODE,
        DL_DMA_NORMAL_MODE, DL_DMA_WIDTH_BYTE, DL_DMA_WIDTH_BYTE,
        DL_DMA_ADDR_UNCHANGED, DL_DMA_ADDR_UNCHANGED);
}

static void BSP_SPI_StopGyroDMA(void)
{
    DL_SPI_disableDMAReceiveEvent(SPI_GYRO_INST, DL_SPI_DMA_INTERRUPT_RX);
    DL_SPI_disableDMATransmitEvent(SPI_GYRO_INST);
    DL_DMA_disableChannel(DMA, BSP_SPI_GYRO_RX_DMA_CHANNEL);
    DL_DMA_disableChannel(DMA, BSP_SPI_GYRO_TX_DMA_CHANNEL);
}

static void BSP_SPI_StartGyroDMA(
    const uint8_t *txData, uint8_t *rxData, size_t length)
{
    const uint8_t *txBuffer = txData;
    uint8_t *rxBuffer = rxData;

    if (txBuffer == NULL) {
        g_spiGyroDummyTx = 0U;
        txBuffer = &g_spiGyroDummyTx;
    }
    if (rxBuffer == NULL) {
        g_spiGyroDummyRx = 0U;
        rxBuffer = &g_spiGyroDummyRx;
    }

    DL_DMA_disableChannel(DMA, BSP_SPI_GYRO_RX_DMA_CHANNEL);
    DL_DMA_disableChannel(DMA, BSP_SPI_GYRO_TX_DMA_CHANNEL);

    DL_DMA_setSrcAddr(
        DMA, BSP_SPI_GYRO_RX_DMA_CHANNEL, (uint32_t) &SPI_GYRO_INST->RXDATA);
    DL_DMA_setDestAddr(
        DMA, BSP_SPI_GYRO_RX_DMA_CHANNEL, (uint32_t) rxBuffer);
    DL_DMA_setSrcIncrement(
        DMA, BSP_SPI_GYRO_RX_DMA_CHANNEL, DL_DMA_ADDR_UNCHANGED);
    DL_DMA_setDestIncrement(DMA, BSP_SPI_GYRO_RX_DMA_CHANNEL,
        (rxData != NULL) ? DL_DMA_ADDR_INCREMENT : DL_DMA_ADDR_UNCHANGED);
    DL_DMA_setTransferSize(
        DMA, BSP_SPI_GYRO_RX_DMA_CHANNEL, (uint16_t) length);

    DL_DMA_setSrcAddr(
        DMA, BSP_SPI_GYRO_TX_DMA_CHANNEL, (uint32_t) txBuffer);
    DL_DMA_setDestAddr(
        DMA, BSP_SPI_GYRO_TX_DMA_CHANNEL, (uint32_t) &SPI_GYRO_INST->TXDATA);
    DL_DMA_setSrcIncrement(DMA, BSP_SPI_GYRO_TX_DMA_CHANNEL,
        (txData != NULL) ? DL_DMA_ADDR_INCREMENT : DL_DMA_ADDR_UNCHANGED);
    DL_DMA_setDestIncrement(
        DMA, BSP_SPI_GYRO_TX_DMA_CHANNEL, DL_DMA_ADDR_UNCHANGED);
    DL_DMA_setTransferSize(
        DMA, BSP_SPI_GYRO_TX_DMA_CHANNEL, (uint16_t) length);

    DL_SPI_clearDMAReceiveEventStatus(
        SPI_GYRO_INST, DL_SPI_DMA_INTERRUPT_RX);
    DL_SPI_clearDMATransmitEventStatus(SPI_GYRO_INST);
    DL_SPI_enableDMAReceiveEvent(SPI_GYRO_INST, DL_SPI_DMA_INTERRUPT_RX);
    DL_SPI_enableDMATransmitEvent(SPI_GYRO_INST);

    DL_DMA_enableChannel(DMA, BSP_SPI_GYRO_RX_DMA_CHANNEL);
    DL_DMA_enableChannel(DMA, BSP_SPI_GYRO_TX_DMA_CHANNEL);
}

static bool BSP_SPI_IsGyroDMAActive(void)
{
    return DL_DMA_isChannelEnabled(DMA, BSP_SPI_GYRO_RX_DMA_CHANNEL) ||
           DL_DMA_isChannelEnabled(DMA, BSP_SPI_GYRO_TX_DMA_CHANNEL);
}

void BSP_SPI_Init(void)
{
    g_spiGyroBusy = false;
    g_spiGyroDummyTx = 0U;
    g_spiGyroDummyRx = 0U;
    DL_GPIO_setPins(
        GPIO_GRP_GYRO_PORT, GPIO_GRP_GYRO_GYRO_CS_N_PIN);
    BSP_SPI_DrainRxFIFO();
    DL_SPI_clearInterruptStatus(SPI_GYRO_INST, BSP_SPI_ERROR_MASK);
    DL_SPI_setBitRateSerialClockDivider(
        SPI_GYRO_INST, BSP_SPI_GYRO_CLOCK_DIVIDER);
    DL_SPI_setFIFOThreshold(SPI_GYRO_INST, DL_SPI_RX_FIFO_LEVEL_ONE_FRAME,
        DL_SPI_TX_FIFO_LEVEL_ONE_FRAME);
    BSP_SPI_ConfigureGyroDMAChannel(
        BSP_SPI_GYRO_RX_DMA_CHANNEL, BSP_SPI_GYRO_RX_DMA_TRIGGER);
    BSP_SPI_ConfigureGyroDMAChannel(
        BSP_SPI_GYRO_TX_DMA_CHANNEL, BSP_SPI_GYRO_TX_DMA_TRIGGER);
    BSP_SPI_StopGyroDMA();
}

BSP_SPI_Status BSP_SPI_GyroTransfer(
    const uint8_t *txData, uint8_t *rxData, size_t length)
{
    uint32_t startMs;
    uint32_t timeoutMs;
    BSP_SPI_Status status = BSP_SPI_STATUS_OK;

    if ((length == 0U) || (length > UINT16_MAX) ||
        ((txData == NULL) && (rxData == NULL))) {
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

    BSP_SPI_StartGyroDMA(txData, rxData, length);
    startMs = BSP_GetTickMs();
    while (BSP_SPI_IsGyroDMAActive()) {
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

    BSP_SPI_StopGyroDMA();

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
