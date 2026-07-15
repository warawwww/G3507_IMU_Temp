#include "heater.h"

#include "bsp_pwm.h"

void Heater_Init(void)
{
    (void) BSP_PWM_SetDutyPermille(0U);
}

bool Heater_SetDutyPermille(uint16_t dutyPermille)
{
    return BSP_PWM_SetDutyPermille(dutyPermille);
}

void Heater_Off(void)
{
    (void) BSP_PWM_SetDutyPermille(0U);
}

uint16_t Heater_GetDutyPermille(void)
{
    return BSP_PWM_GetDutyPermille();
}
