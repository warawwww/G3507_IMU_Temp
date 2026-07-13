#ifndef BSP_SPI_H
#define BSP_SPI_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    BSP_SPI_STATUS_OK = 0,
    BSP_SPI_STATUS_INVALID_ARGUMENT,
    BSP_SPI_STATUS_BUSY,
    BSP_SPI_STATUS_TIMEOUT,
    BSP_SPI_STATUS_TRANSFER_ERROR,
} BSP_SPI_Status;

/* Called by BSP_Init(); application code normally does not call this. */
void BSP_SPI_Init(void);

/*
 * Full-duplex transfer on the gyro SPI bus. The gyro CS pin remains low for
 * the complete transaction. Pass NULL for txData to transmit zeroes, or NULL
 * for rxData to discard received bytes. Do not call from an ISR.
 */
BSP_SPI_Status BSP_SPI_GyroTransfer(
    const uint8_t *txData, uint8_t *rxData, size_t length);

#endif
