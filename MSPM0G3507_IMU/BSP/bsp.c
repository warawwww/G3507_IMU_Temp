#include "bsp.h"

#include "ti_msp_dl_config.h"

void BSP_Init(void)
{
    SYSCFG_DL_init();
}

void BSP_Idle(void)
{
    __WFI();
}
