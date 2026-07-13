#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdbool.h>
#include <stdint.h>

/** 初始化 PWM 输出，并将其强制保持为低电平。 */
void BSP_PWM_Init(void);

/**
 * 设置 PWM 输出占空比，范围为 0~100%。
 *
 * 0% 会停止定时器并强制输出低电平。
 */
bool BSP_PWM_SetDutyPercent(uint8_t dutyPercent);

/** 返回当前设置的 PWM 占空比。 */
uint8_t BSP_PWM_GetDutyPercent(void);

#endif
