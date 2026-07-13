#ifndef USER_HEATER_TASK_H
#define USER_HEATER_TASK_H

#include <stdbool.h>
#include <stdint.h>

/** 初始化加热任务，加热输出默认为关闭。 */
void Heater_Task_Init(void);

/** 处理按键、温度闭环和加热状态上报，需要在主循环中周期调用。 */
void Heater_Task_Run(void);

/** 返回当前是否已启用温度闭环。 */
bool Heater_Task_IsEnabled(void);

/** 返回当前加热 PWM 占空比。 */
uint8_t Heater_Task_GetDutyPercent(void);

/** 返回当前加热 PWM 占空比，单位为 0.1%。 */
uint16_t Heater_Task_GetDutyPermille(void);

#endif
