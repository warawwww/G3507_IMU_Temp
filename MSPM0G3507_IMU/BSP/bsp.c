#include "bsp.h"

#include "bsp_i2c.h"
#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "bsp_uart.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_tickMs;

void SysTick_Handler(void);

void BSP_Init(void)
{
    g_tickMs = 0U;
    SYSCFG_DL_init();
    BSP_PWM_Init();
    BSP_I2C_Init();
    BSP_SPI_Init();
    BSP_UART_Init();
}

uint32_t BSP_GetTickMs(void)
{
    return g_tickMs;
}

void SysTick_Handler(void)
{
    g_tickMs++;
}
