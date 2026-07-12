#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    BSP_UART_PORT_TYPEC = 0,
    BSP_UART_PORT_EXTERNAL,
} BSP_UART_Port;

/* UART hardware is initialized by BSP_Init() through SYSCFG_DL_init(). */
bool BSP_UART_WriteByte(BSP_UART_Port port, uint8_t data);
bool BSP_UART_Write(BSP_UART_Port port, const uint8_t *data, size_t length);
bool BSP_UART_WriteString(BSP_UART_Port port, const char *string);

/* Returns immediately. false means that no byte is currently available. */
bool BSP_UART_TryReadByte(BSP_UART_Port port, uint8_t *data);

/* Waits until one byte is received. */
bool BSP_UART_ReadByteBlocking(BSP_UART_Port port, uint8_t *data);

#endif
