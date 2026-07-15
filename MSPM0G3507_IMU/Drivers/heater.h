#ifndef DRIVERS_HEATER_H
#define DRIVERS_HEATER_H

#include <stdbool.h>
#include <stdint.h>

/** Initialize heater output, off by default. */
void Heater_Init(void);

/** Set heater PWM duty in 0.1% units, range 0~1000. */
bool Heater_SetDutyPermille(uint16_t dutyPermille);

/** Turn off heater output. */
void Heater_Off(void);

/** Return the current heater PWM duty in 0.1% units. */
uint16_t Heater_GetDutyPermille(void);

#endif
