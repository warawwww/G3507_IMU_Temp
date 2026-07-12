#include "bsp_uart.h"

#include <string.h>

#include "ti_msp_dl_config.h"

typedef struct {
    UART_Regs *uart;
    uint8_t txDmaChannel;
    uint8_t rxDmaChannel;
    uint8_t txBuffer[BSP_UART_TX_BUFFER_SIZE];
    uint8_t rxBuffer[BSP_UART_RX_BUFFER_SIZE];
    volatile bool txBusy;
    volatile bool rxOverflow;
    volatile uint32_t rxCompletedCount;
    uint32_t rxReadCount;
} BSP_UART_State;

static BSP_UART_State g_uartState[] = {
    {
        .uart         = UART_TYPEC_INST,
        .txDmaChannel = DMA_TYPEC_TX_CHAN_ID,
        .rxDmaChannel = DMA_TYPEC_RX_CHAN_ID,
    },
    {
        .uart         = UART_EXTERNAL_INST,
        .txDmaChannel = DMA_EXTERNAL_TX_CHAN_ID,
        .rxDmaChannel = DMA_EXTERNAL_RX_CHAN_ID,
    },
};

static BSP_UART_State *BSP_UART_GetState(BSP_UART_Port port);
static void BSP_UART_StartRxDMA(BSP_UART_State *state);
static uint32_t BSP_UART_GetRxProduced(BSP_UART_State *state);
static void BSP_UART_HandleInterrupt(BSP_UART_Port port);

void UART_TYPEC_INST_IRQHandler(void);
void UART_EXTERNAL_INST_IRQHandler(void);

static BSP_UART_State *BSP_UART_GetState(BSP_UART_Port port)
{
    if ((uint32_t) port >= (sizeof(g_uartState) / sizeof(g_uartState[0]))) {
        return NULL;
    }

    return &g_uartState[port];
}

static void BSP_UART_StartRxDMA(BSP_UART_State *state)
{
    DL_DMA_disableChannel(DMA, state->rxDmaChannel);
    DL_DMA_setSrcAddr(
        DMA, state->rxDmaChannel, (uint32_t) &state->uart->RXDATA);
    DL_DMA_setDestAddr(
        DMA, state->rxDmaChannel, (uint32_t) &state->rxBuffer[0]);
    DL_DMA_setTransferSize(
        DMA, state->rxDmaChannel, BSP_UART_RX_BUFFER_SIZE);
    DL_DMA_enableChannel(DMA, state->rxDmaChannel);
}

void BSP_UART_Init(void)
{
    for (size_t index = 0U;
         index < (sizeof(g_uartState) / sizeof(g_uartState[0])); index++) {
        g_uartState[index].txBusy           = false;
        g_uartState[index].rxOverflow       = false;
        g_uartState[index].rxCompletedCount = 0U;
        g_uartState[index].rxReadCount      = 0U;
        BSP_UART_StartRxDMA(&g_uartState[index]);
    }

    NVIC_ClearPendingIRQ(UART_TYPEC_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_EXTERNAL_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_TYPEC_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_EXTERNAL_INST_INT_IRQN);
}

bool BSP_UART_WriteByte(BSP_UART_Port port, uint8_t data)
{
    return BSP_UART_Write(port, &data, 1U);
}

bool BSP_UART_Write(BSP_UART_Port port, const uint8_t *data, size_t length)
{
    BSP_UART_State *state = BSP_UART_GetState(port);
    uint32_t primask;

    if ((state == NULL) || ((data == NULL) && (length != 0U)) ||
        (length > BSP_UART_TX_BUFFER_SIZE)) {
        return false;
    }

    if (length == 0U) {
        return true;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (state->txBusy) {
        if (primask == 0U) {
            __enable_irq();
        }
        return false;
    }
    state->txBusy = true;
    if (primask == 0U) {
        __enable_irq();
    }

    memcpy(state->txBuffer, data, length);

    DL_DMA_disableChannel(DMA, state->txDmaChannel);
    DL_DMA_setSrcAddr(
        DMA, state->txDmaChannel, (uint32_t) &state->txBuffer[0]);
    DL_DMA_setDestAddr(
        DMA, state->txDmaChannel, (uint32_t) &state->uart->TXDATA);
    DL_DMA_setTransferSize(DMA, state->txDmaChannel, (uint16_t) length);
    DL_DMA_enableChannel(DMA, state->txDmaChannel);

    return true;
}

bool BSP_UART_WriteString(BSP_UART_Port port, const char *string)
{
    size_t length = 0U;

    if (string == NULL) {
        return false;
    }

    while ((length < BSP_UART_TX_BUFFER_SIZE) &&
           (string[length] != '\0')) {
        length++;
    }

    if (string[length] != '\0') {
        return false;
    }

    return BSP_UART_Write(port, (const uint8_t *) string, length);
}

bool BSP_UART_IsTxBusy(BSP_UART_Port port)
{
    BSP_UART_State *state = BSP_UART_GetState(port);

    return (state != NULL) && state->txBusy;
}

static uint32_t BSP_UART_GetRxProduced(BSP_UART_State *state)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t completed;
    uint16_t remaining;

    __disable_irq();
    completed = state->rxCompletedCount;
    remaining = DL_DMA_getTransferSize(DMA, state->rxDmaChannel);
    if (primask == 0U) {
        __enable_irq();
    }

    return completed + (BSP_UART_RX_BUFFER_SIZE - remaining);
}

size_t BSP_UART_RxAvailable(BSP_UART_Port port)
{
    BSP_UART_State *state = BSP_UART_GetState(port);
    uint32_t produced;
    uint32_t available;

    if (state == NULL) {
        return 0U;
    }

    produced  = BSP_UART_GetRxProduced(state);
    available = produced - state->rxReadCount;

    if (available > BSP_UART_RX_BUFFER_SIZE) {
        state->rxReadCount = produced - BSP_UART_RX_BUFFER_SIZE;
        state->rxOverflow  = true;
        available          = BSP_UART_RX_BUFFER_SIZE;
    }

    return (size_t) available;
}

bool BSP_UART_TryReadByte(BSP_UART_Port port, uint8_t *data)
{
    return BSP_UART_Read(port, data, 1U) == 1U;
}

size_t BSP_UART_Read(BSP_UART_Port port, uint8_t *data, size_t length)
{
    BSP_UART_State *state = BSP_UART_GetState(port);
    size_t available;
    size_t readLength;

    if ((state == NULL) || ((data == NULL) && (length != 0U))) {
        return 0U;
    }

    available  = BSP_UART_RxAvailable(port);
    readLength = (length < available) ? length : available;

    for (size_t index = 0U; index < readLength; index++) {
        data[index] = state->rxBuffer[
            (state->rxReadCount + index) % BSP_UART_RX_BUFFER_SIZE];
    }
    state->rxReadCount += (uint32_t) readLength;

    return readLength;
}

void BSP_UART_FlushRx(BSP_UART_Port port)
{
    BSP_UART_State *state = BSP_UART_GetState(port);

    if (state != NULL) {
        state->rxReadCount = BSP_UART_GetRxProduced(state);
    }
}

bool BSP_UART_RxOverflowed(BSP_UART_Port port)
{
    BSP_UART_State *state = BSP_UART_GetState(port);

    (void) BSP_UART_RxAvailable(port);
    return (state != NULL) && state->rxOverflow;
}

void BSP_UART_ClearRxOverflow(BSP_UART_Port port)
{
    BSP_UART_State *state = BSP_UART_GetState(port);

    if (state != NULL) {
        state->rxOverflow = false;
    }
}

static void BSP_UART_HandleInterrupt(BSP_UART_Port port)
{
    BSP_UART_State *state = BSP_UART_GetState(port);
    DL_UART_IIDX interruptIndex;

    if (state == NULL) {
        return;
    }

    do {
        interruptIndex = DL_UART_Main_getPendingInterrupt(state->uart);

        switch (interruptIndex) {
            case DL_UART_MAIN_IIDX_DMA_DONE_RX:
                state->rxCompletedCount += BSP_UART_RX_BUFFER_SIZE;
                BSP_UART_StartRxDMA(state);
                break;

            case DL_UART_MAIN_IIDX_DMA_DONE_TX:
                state->txBusy = false;
                break;

            default:
                break;
        }
    } while (interruptIndex != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}

void UART_TYPEC_INST_IRQHandler(void)
{
    BSP_UART_HandleInterrupt(BSP_UART_PORT_TYPEC);
}

void UART_EXTERNAL_INST_IRQHandler(void)
{
    BSP_UART_HandleInterrupt(BSP_UART_PORT_EXTERNAL);
}
