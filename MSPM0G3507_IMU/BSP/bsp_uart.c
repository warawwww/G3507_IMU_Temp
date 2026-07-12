#include "bsp_uart.h"

#include "ti_msp_dl_config.h"

static UART_Regs *BSP_UART_GetInstance(BSP_UART_Port port)
{
    switch (port) {
        case BSP_UART_PORT_TYPEC:
            return UART_TYPEC_INST;

        case BSP_UART_PORT_EXTERNAL:
            return UART_EXTERNAL_INST;

        default:
            return NULL;
    }
}

bool BSP_UART_WriteByte(BSP_UART_Port port, uint8_t data)
{
    UART_Regs *uart = BSP_UART_GetInstance(port);

    if (uart == NULL) {
        return false;
    }

    DL_UART_Main_transmitDataBlocking(uart, data);
    return true;
}

bool BSP_UART_Write(BSP_UART_Port port, const uint8_t *data, size_t length)
{
    UART_Regs *uart = BSP_UART_GetInstance(port);

    if ((uart == NULL) || ((data == NULL) && (length != 0U))) {
        return false;
    }

    for (size_t index = 0U; index < length; index++) {
        DL_UART_Main_transmitDataBlocking(uart, data[index]);
    }

    return true;
}

bool BSP_UART_WriteString(BSP_UART_Port port, const char *string)
{
    UART_Regs *uart = BSP_UART_GetInstance(port);

    if ((uart == NULL) || (string == NULL)) {
        return false;
    }

    while (*string != '\0') {
        DL_UART_Main_transmitDataBlocking(uart, (uint8_t) *string);
        string++;
    }

    return true;
}

bool BSP_UART_TryReadByte(BSP_UART_Port port, uint8_t *data)
{
    UART_Regs *uart = BSP_UART_GetInstance(port);

    if ((uart == NULL) || (data == NULL)) {
        return false;
    }

    return DL_UART_Main_receiveDataCheck(uart, data);
}

bool BSP_UART_ReadByteBlocking(BSP_UART_Port port, uint8_t *data)
{
    UART_Regs *uart = BSP_UART_GetInstance(port);

    if ((uart == NULL) || (data == NULL)) {
        return false;
    }

    *data = DL_UART_Main_receiveDataBlocking(uart);
    return true;
}
