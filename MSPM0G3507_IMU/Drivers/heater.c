#include "heater.h"

#include "bsp_pwm.h"

void Heater_Init(void)
{
    (void) BSP_PWM_SetDutyPercent(0U);
}

bool Heater_SetDutyPercent(uint8_t dutyPercent)
{
    return BSP_PWM_SetDutyPercent(dutyPercent);
}

void Heater_Off(void)
{
    (void) BSP_PWM_SetDutyPercent(0U);
}

uint8_t Heater_GetDutyPercent(void)
{
    return BSP_PWM_GetDutyPercent();
}

bool Heater_IsEnabled(void)
{
    return Heater_GetDutyPercent() != 0U;
}
