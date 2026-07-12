#include "bsp.h"

#include "ti_msp_dl_config.h"

static volatile uint32_t g_tickMs;

void SysTick_Handler(void);

void BSP_Init(void)
{
    g_tickMs = 0U;
    SYSCFG_DL_init();
}

uint32_t BSP_GetTickMs(void)
{
    return g_tickMs;
}

void SysTick_Handler(void)
{
    g_tickMs++;
}
