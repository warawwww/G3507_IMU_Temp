#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BSP_UART_TX_BUFFER_SIZE (256U)
#define BSP_UART_RX_BUFFER_SIZE (256U)

typedef enum {
    BSP_UART_PORT_TYPEC = 0,
    BSP_UART_PORT_EXTERNAL,
} BSP_UART_Port;

/* Called by BSP_Init(); application code normally does not call this. */
void BSP_UART_Init(void);

/*
 * Starts an asynchronous DMA transmission and returns immediately.
 * The data is copied into a BSP-owned buffer. false means invalid arguments,
 * the port is busy, or length exceeds BSP_UART_TX_BUFFER_SIZE.
 */
bool BSP_UART_WriteByte(BSP_UART_Port port, uint8_t data);
bool BSP_UART_Write(BSP_UART_Port port, const uint8_t *data, size_t length);
bool BSP_UART_WriteString(BSP_UART_Port port, const char *string);
bool BSP_UART_IsTxBusy(BSP_UART_Port port);

/* DMA continuously feeds an internal RX ring buffer. */
size_t BSP_UART_RxAvailable(BSP_UART_Port port);
bool BSP_UART_TryReadByte(BSP_UART_Port port, uint8_t *data);
size_t BSP_UART_Read(BSP_UART_Port port, uint8_t *data, size_t length);
void BSP_UART_FlushRx(BSP_UART_Port port);

/* An overflow discards the oldest unread bytes and retains the newest data. */
bool BSP_UART_RxOverflowed(BSP_UART_Port port);
void BSP_UART_ClearRxOverflow(BSP_UART_Port port);

#endif
