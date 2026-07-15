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

uint32_t BSP_GetTickUs(void)
{
    uint32_t tickMs;
    uint32_t tickMsCheck;
    uint32_t reloadTicks;
    uint32_t currentTicks;
    uint32_t elapsedTicks;

    do {
        tickMs = g_tickMs;
        currentTicks = SysTick->VAL;
        tickMsCheck = g_tickMs;
    } while (tickMs != tickMsCheck);

    reloadTicks = SysTick->LOAD + 1U;
    elapsedTicks = reloadTicks - currentTicks;
    if (elapsedTicks > reloadTicks) {
        elapsedTicks = 0U;
    }

    return (tickMs * 1000U) +
           (uint32_t) (((uint64_t) elapsedTicks * 1000U) / reloadTicks);
}

void SysTick_Handler(void)
{
    g_tickMs++;
}
