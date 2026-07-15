#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdbool.h>
#include <stdint.h>

/** Initialize PWM output and force it low by default. */
void BSP_PWM_Init(void);

/** Set PWM duty in 0.1% units, range 0~1000. */
bool BSP_PWM_SetDutyPermille(uint16_t dutyPermille);

/** Return the current PWM duty in 0.1% units. */
uint16_t BSP_PWM_GetDutyPermille(void);

#endif
