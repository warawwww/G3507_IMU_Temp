#ifndef BSP_I2C_H
#define BSP_I2C_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    BSP_I2C_STATUS_OK = 0,
    BSP_I2C_STATUS_INVALID_ARGUMENT,
    BSP_I2C_STATUS_BUSY,
    BSP_I2C_STATUS_TIMEOUT,
    BSP_I2C_STATUS_NACK,
    BSP_I2C_STATUS_ARBITRATION_LOST,
    BSP_I2C_STATUS_TRANSFER_ERROR,
} BSP_I2C_Status;

/* Called by BSP_Init(); application code normally does not call this. */
void BSP_I2C_Init(void);

/* These blocking APIs must be called from main context, not from an ISR. */
BSP_I2C_Status BSP_I2C_Write(
    uint8_t address, const uint8_t *data, size_t length);
BSP_I2C_Status BSP_I2C_Read(
    uint8_t address, uint8_t *data, size_t length);

/* Performs write + repeated START + read without releasing the bus. */
BSP_I2C_Status BSP_I2C_WriteRead(uint8_t address,
    const uint8_t *writeData, size_t writeLength, uint8_t *readData,
    size_t readLength);

#endif
