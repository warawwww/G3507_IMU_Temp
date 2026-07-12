#ifndef DRIVERS_KEY_H
#define DRIVERS_KEY_H

#include <stdbool.h>

#define KEY_DEBOUNCE_TIME_MS   (20U)
#define KEY_LONG_PRESS_TIME_MS (500U)

/** 初始化用户按键状态和消抖器。 */
void KEY_Init(void);

/**
 * 更新按键消抖状态。
 *
 * 需要由主循环周期性调用。
 */
void KEY_Update(void);

/** 返回消抖后的当前按下状态。 */
bool KEY_IsPressed(void);

/**
 * 获取一次按下事件。
 *
 * 返回 true 后会清除本次事件。
 */
bool KEY_WasPressed(void);

/**
 * 获取一次释放事件。
 *
 * 返回 true 后会清除本次事件。
 */
bool KEY_WasReleased(void);

/**
 * 获取一次短按事件。
 *
 * 按键稳定按下但未达到长按阈值，释放后产生该事件。
 */
bool KEY_WasShortPressed(void);

/**
 * 获取一次长按事件。
 *
 * 按住达到 KEY_LONG_PRESS_TIME_MS 后产生一次，不必等待释放。
 */
bool KEY_WasLongPressed(void);

#endif
