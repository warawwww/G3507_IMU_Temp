#ifndef DRIVERS_HEATER_H
#define DRIVERS_HEATER_H

#include <stdbool.h>
#include <stdint.h>

/** 初始化加热开关，默认关闭。 */
void Heater_Init(void);

/** 设置加热功率占空比，范围为 0~100%。 */
bool Heater_SetDutyPercent(uint8_t dutyPercent);

/** 以 0.1% 为单位设置加热占空比，范围为 0~1000。 */
bool Heater_SetDutyPermille(uint16_t dutyPermille);

/** 关闭加热输出。 */
void Heater_Off(void);

/** 返回当前加热占空比。 */
uint8_t Heater_GetDutyPercent(void);

/** 返回当前加热占空比，单位为 0.1%。 */
uint16_t Heater_GetDutyPermille(void);

/** 返回当前是否正在输出加热 PWM。 */
bool Heater_IsEnabled(void);

#endif
