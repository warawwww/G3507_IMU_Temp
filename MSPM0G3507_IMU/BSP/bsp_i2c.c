#include "bsp_i2c.h"

#include <stdbool.h>

#include "bsp.h"
#include "ti_msp_dl_config.h"

#define BSP_I2C_MAX_TRANSFER_SIZE (0x0FFFU)
#define BSP_I2C_TIMEOUT_MARGIN_MS  (5U)

#define BSP_I2C_INTERRUPT_MASK                                            \
    (DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |                       \
        DL_I2C_INTERRUPT_CONTROLLER_NACK |                                \
        DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |                      \
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |                             \
        DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER |                      \
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE)

typedef enum {
    BSP_I2C_TRANSFER_IDLE = 0,
    BSP_I2C_TRANSFER_TX,
    BSP_I2C_TRANSFER_RX,
    BSP_I2C_TRANSFER_COMPLETE,
    BSP_I2C_TRANSFER_ERROR,
} BSP_I2C_TransferState;

typedef struct {
    const uint8_t *txData;
    uint8_t *rxData;
    size_t txLength;
    size_t rxLength;
    size_t txCount;
    size_t rxCount;
    uint8_t address;
    volatile BSP_I2C_TransferState state;
    volatile BSP_I2C_Status result;
} BSP_I2C_Transaction;

static BSP_I2C_Transaction g_i2cTransaction;

static void BSP_I2C_DrainRxFIFO(void);
static uint32_t BSP_I2C_GetTimeoutMs(size_t byteCount);
static BSP_I2C_Status BSP_I2C_Transfer(uint8_t address,
    const uint8_t *writeData, size_t writeLength, uint8_t *readData,
    size_t readLength);

void I2C_TEMP_INST_IRQHandler(void);

void BSP_I2C_Init(void)
{
    g_i2cTransaction.state  = BSP_I2C_TRANSFER_IDLE;
    g_i2cTransaction.result = BSP_I2C_STATUS_OK;

    DL_I2C_resetControllerTransfer(I2C_TEMP_INST);
    DL_I2C_flushControllerTXFIFO(I2C_TEMP_INST);
    DL_I2C_flushControllerRXFIFO(I2C_TEMP_INST);
    DL_I2C_disableInterrupt(
        I2C_TEMP_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    DL_I2C_clearInterruptStatus(I2C_TEMP_INST, BSP_I2C_INTERRUPT_MASK);

    NVIC_ClearPendingIRQ(I2C_TEMP_INST_INT_IRQN);
    NVIC_EnableIRQ(I2C_TEMP_INST_INT_IRQN);
}

static uint32_t BSP_I2C_GetTimeoutMs(size_t byteCount)
{
    uint32_t wireTimeMs = (uint32_t) (((byteCount + 2U) * 10U * 1000U +
                                          I2C_TEMP_BUS_SPEED_HZ - 1U) /
                                      I2C_TEMP_BUS_SPEED_HZ);

    return wireTimeMs + BSP_I2C_TIMEOUT_MARGIN_MS;
}

static BSP_I2C_Status BSP_I2C_Transfer(uint8_t address,
    const uint8_t *writeData, size_t writeLength, uint8_t *readData,
    size_t readLength)
{
    uint32_t startMs;
    uint32_t timeoutMs;
    uint16_t initiallyFilled;

    if ((address > 0x7FU) || (writeLength > BSP_I2C_MAX_TRANSFER_SIZE) ||
        (readLength > BSP_I2C_MAX_TRANSFER_SIZE) ||
        ((writeLength == 0U) && (readLength == 0U)) ||
        ((writeData == NULL) && (writeLength != 0U)) ||
        ((readData == NULL) && (readLength != 0U))) {
        return BSP_I2C_STATUS_INVALID_ARGUMENT;
    }

    if (g_i2cTransaction.state != BSP_I2C_TRANSFER_IDLE) {
        return BSP_I2C_STATUS_BUSY;
    }

    timeoutMs = BSP_I2C_GetTimeoutMs(writeLength + readLength);
    startMs   = BSP_GetTickMs();
    while ((DL_I2C_getControllerStatus(I2C_TEMP_INST) &
               DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if ((uint32_t) (BSP_GetTickMs() - startMs) >= timeoutMs) {
            return BSP_I2C_STATUS_BUSY;
        }
    }

    DL_I2C_flushControllerTXFIFO(I2C_TEMP_INST);
    DL_I2C_flushControllerRXFIFO(I2C_TEMP_INST);
    DL_I2C_clearInterruptStatus(I2C_TEMP_INST, BSP_I2C_INTERRUPT_MASK);

    g_i2cTransaction.txData   = writeData;
    g_i2cTransaction.rxData   = readData;
    g_i2cTransaction.txLength = writeLength;
    g_i2cTransaction.rxLength = readLength;
    g_i2cTransaction.txCount  = 0U;
    g_i2cTransaction.rxCount  = 0U;
    g_i2cTransaction.address  = address;
    g_i2cTransaction.result   = BSP_I2C_STATUS_OK;

    if (writeLength != 0U) {
        initiallyFilled = DL_I2C_fillControllerTXFIFO(I2C_TEMP_INST,
            writeData, (uint16_t) writeLength);
        g_i2cTransaction.txCount = initiallyFilled;
        g_i2cTransaction.state   = BSP_I2C_TRANSFER_TX;

        if (g_i2cTransaction.txCount < g_i2cTransaction.txLength) {
            DL_I2C_enableInterrupt(
                I2C_TEMP_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        } else {
            DL_I2C_disableInterrupt(
                I2C_TEMP_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        }

        DL_I2C_startControllerTransferAdvanced(I2C_TEMP_INST, address,
            DL_I2C_CONTROLLER_DIRECTION_TX, (uint16_t) writeLength,
            DL_I2C_CONTROLLER_START_ENABLE,
            (readLength == 0U) ? DL_I2C_CONTROLLER_STOP_ENABLE
                               : DL_I2C_CONTROLLER_STOP_DISABLE,
            DL_I2C_CONTROLLER_ACK_DISABLE);
    } else {
        g_i2cTransaction.state = BSP_I2C_TRANSFER_RX;
        DL_I2C_startControllerTransferAdvanced(I2C_TEMP_INST, address,
            DL_I2C_CONTROLLER_DIRECTION_RX, (uint16_t) readLength,
            DL_I2C_CONTROLLER_START_ENABLE, DL_I2C_CONTROLLER_STOP_ENABLE,
            DL_I2C_CONTROLLER_ACK_DISABLE);
    }

    startMs = BSP_GetTickMs();
    while ((g_i2cTransaction.state != BSP_I2C_TRANSFER_COMPLETE) &&
           (g_i2cTransaction.state != BSP_I2C_TRANSFER_ERROR)) {
        if ((uint32_t) (BSP_GetTickMs() - startMs) >= timeoutMs) {
            g_i2cTransaction.result = BSP_I2C_STATUS_TIMEOUT;
            g_i2cTransaction.state  = BSP_I2C_TRANSFER_ERROR;
            DL_I2C_resetControllerTransfer(I2C_TEMP_INST);
            DL_I2C_flushControllerTXFIFO(I2C_TEMP_INST);
            DL_I2C_flushControllerRXFIFO(I2C_TEMP_INST);
            break;
        }
    }

    BSP_I2C_Status result = g_i2cTransaction.result;

    NVIC_DisableIRQ(I2C_TEMP_INST_INT_IRQN);
    DL_I2C_disableInterrupt(
        I2C_TEMP_INST, DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
    if (result != BSP_I2C_STATUS_OK) {
        DL_I2C_resetControllerTransfer(I2C_TEMP_INST);
        DL_I2C_flushControllerTXFIFO(I2C_TEMP_INST);
        DL_I2C_flushControllerRXFIFO(I2C_TEMP_INST);
    }
    DL_I2C_clearInterruptStatus(I2C_TEMP_INST, BSP_I2C_INTERRUPT_MASK);
    g_i2cTransaction.state = BSP_I2C_TRANSFER_IDLE;
    NVIC_ClearPendingIRQ(I2C_TEMP_INST_INT_IRQN);
    NVIC_EnableIRQ(I2C_TEMP_INST_INT_IRQN);

    return result;
}

BSP_I2C_Status BSP_I2C_Write(
    uint8_t address, const uint8_t *data, size_t length)
{
    return BSP_I2C_Transfer(address, data, length, NULL, 0U);
}

BSP_I2C_Status BSP_I2C_Read(uint8_t address, uint8_t *data, size_t length)
{
    return BSP_I2C_Transfer(address, NULL, 0U, data, length);
}

BSP_I2C_Status BSP_I2C_WriteRead(uint8_t address,
    const uint8_t *writeData, size_t writeLength, uint8_t *readData,
    size_t readLength)
{
    if ((writeLength == 0U) || (readLength == 0U)) {
        return BSP_I2C_STATUS_INVALID_ARGUMENT;
    }

    return BSP_I2C_Transfer(
        address, writeData, writeLength, readData, readLength);
}

static void BSP_I2C_DrainRxFIFO(void)
{
    while (!DL_I2C_isControllerRXFIFOEmpty(I2C_TEMP_INST)) {
        uint8_t data = DL_I2C_receiveControllerData(I2C_TEMP_INST);

        if (g_i2cTransaction.rxCount < g_i2cTransaction.rxLength) {
            g_i2cTransaction.rxData[g_i2cTransaction.rxCount] = data;
            g_i2cTransaction.rxCount++;
        }
    }
}

void I2C_TEMP_INST_IRQHandler(void)
{
    DL_I2C_IIDX interruptIndex;

    do {
        interruptIndex = DL_I2C_getPendingInterrupt(I2C_TEMP_INST);

        switch (interruptIndex) {
            case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
                if (g_i2cTransaction.txCount < g_i2cTransaction.txLength) {
                    uint16_t filled = DL_I2C_fillControllerTXFIFO(I2C_TEMP_INST,
                        &g_i2cTransaction.txData[g_i2cTransaction.txCount],
                        (uint16_t) (g_i2cTransaction.txLength -
                                    g_i2cTransaction.txCount));
                    g_i2cTransaction.txCount += filled;
                }
                if (g_i2cTransaction.txCount == g_i2cTransaction.txLength) {
                    DL_I2C_disableInterrupt(I2C_TEMP_INST,
                        DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
                }
                break;

            case DL_I2C_IIDX_CONTROLLER_TX_DONE:
                DL_I2C_disableInterrupt(I2C_TEMP_INST,
                    DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
                if ((g_i2cTransaction.state == BSP_I2C_TRANSFER_TX) &&
                    (g_i2cTransaction.rxLength != 0U)) {
                    g_i2cTransaction.state = BSP_I2C_TRANSFER_RX;
                    DL_I2C_startControllerTransferAdvanced(I2C_TEMP_INST,
                        g_i2cTransaction.address,
                        DL_I2C_CONTROLLER_DIRECTION_RX,
                        (uint16_t) g_i2cTransaction.rxLength,
                        DL_I2C_CONTROLLER_START_ENABLE,
                        DL_I2C_CONTROLLER_STOP_ENABLE,
                        DL_I2C_CONTROLLER_ACK_DISABLE);
                } else {
                    g_i2cTransaction.state = BSP_I2C_TRANSFER_COMPLETE;
                }
                break;

            case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
                BSP_I2C_DrainRxFIFO();
                break;

            case DL_I2C_IIDX_CONTROLLER_RX_DONE:
                BSP_I2C_DrainRxFIFO();
                if (g_i2cTransaction.rxCount == g_i2cTransaction.rxLength) {
                    g_i2cTransaction.state = BSP_I2C_TRANSFER_COMPLETE;
                } else {
                    g_i2cTransaction.result = BSP_I2C_STATUS_TRANSFER_ERROR;
                    g_i2cTransaction.state  = BSP_I2C_TRANSFER_ERROR;
                }
                break;

            case DL_I2C_IIDX_CONTROLLER_NACK:
                g_i2cTransaction.result = BSP_I2C_STATUS_NACK;
                g_i2cTransaction.state  = BSP_I2C_TRANSFER_ERROR;
                break;

            case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
                g_i2cTransaction.result = BSP_I2C_STATUS_ARBITRATION_LOST;
                g_i2cTransaction.state  = BSP_I2C_TRANSFER_ERROR;
                break;

            default:
                break;
        }
    } while (interruptIndex != DL_I2C_IIDX_NO_INT);
}
