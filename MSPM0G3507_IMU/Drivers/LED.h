#ifndef DRIVERS_LED_H
#define DRIVERS_LED_H

#include <stdbool.h>

/** 板载 LED 标识。 */
typedef enum {
    LED_ID_GREEN = 0,
    LED_ID_RED,
    LED_ID_COUNT
} LED_ID;

/**
 * 初始化 LED 驱动。
 *
 * 应在 BSP_Init() 完成 GPIO 初始化后调用。
 */
void LED_Init(void);

/** 按逻辑状态设置 LED，不向上层暴露高/低电平有效关系。 */
void LED_Set(LED_ID led, bool on);

/** 点亮指定 LED。 */
void LED_On(LED_ID led);

/** 熄灭指定 LED。 */
void LED_Off(LED_ID led);

/** 翻转指定 LED 的状态。 */
void LED_Toggle(LED_ID led);

#endif
