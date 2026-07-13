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

bool Heater_SetDutyPermille(uint16_t dutyPermille)
{
    return BSP_PWM_SetDutyPermille(dutyPermille);
}

void Heater_Off(void)
{
    (void) BSP_PWM_SetDutyPercent(0U);
}

uint8_t Heater_GetDutyPercent(void)
{
    return BSP_PWM_GetDutyPercent();
}

uint16_t Heater_GetDutyPermille(void)
{
    return BSP_PWM_GetDutyPermille();
}

bool Heater_IsEnabled(void)
{
    return Heater_GetDutyPermille() != 0U;
}
