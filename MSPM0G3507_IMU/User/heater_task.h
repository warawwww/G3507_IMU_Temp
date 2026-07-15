#ifndef USER_HEATER_TASK_H
#define USER_HEATER_TASK_H

#include <stdbool.h>
#include <stdint.h>

/** Initialize heater control. Output is off by default. */
void Heater_Task_Init(void);

/** Run temperature control and heater status reporting. */
void Heater_Task_Run(void);

/** Enable closed-loop heating. */
void Heater_Task_Enable(void);

/** Disable closed-loop heating and turn off output. */
void Heater_Task_Disable(void);

/** Return whether closed-loop heating is enabled. */
bool Heater_Task_IsEnabled(void);

/** Return whether the heater has entered the stable hold phase. */
bool Heater_Task_IsStable(void);

/** Return current heater PWM duty in 0.1% units. */
uint16_t Heater_Task_GetDutyPermille(void);

#endif
