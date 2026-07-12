#include "KEY.h"

#include <stdint.h>

#include "bsp.h"
#include "ti_msp_dl_config.h"

static bool g_stablePressed;
static bool g_pressedEvent;
static bool g_releasedEvent;
static bool g_shortPressEvent;
static bool g_longPressEvent;
static bool g_longPressReported;
static uint32_t g_pressStartMs;
static volatile bool g_debouncePending;
static volatile uint32_t g_edgeTimeMs;

void GROUP1_IRQHandler(void);

static bool KEY_ReadRawPressed(void)
{
    uint32_t pinState = DL_GPIO_readPins(GPIO_GRP_KEY_PORT, GPIO_GRP_KEY_KEY_USER_PIN);

    /* PB21 由外部电阻上拉，按键按下时接地。 */
    return (pinState & GPIO_GRP_KEY_KEY_USER_PIN) == 0U;
}

void KEY_Init(void)
{
    bool pressed = KEY_ReadRawPressed();
    uint32_t nowMs = BSP_GetTickMs();

    g_stablePressed   = pressed;
    g_pressedEvent    = false;
    g_releasedEvent   = false;
    g_shortPressEvent = false;
    g_longPressEvent  = false;
    g_longPressReported = false;
    g_pressStartMs      = nowMs;
    g_edgeTimeMs        = nowMs;
    g_debouncePending   = false;

    DL_GPIO_clearInterruptStatus(GPIO_GRP_KEY_PORT, GPIO_GRP_KEY_KEY_USER_PIN);
    NVIC_ClearPendingIRQ(GPIO_GRP_KEY_INT_IRQN);
    NVIC_EnableIRQ(GPIO_GRP_KEY_INT_IRQN);
}

void KEY_Update(void)
{
    uint32_t nowMs = BSP_GetTickMs();

    if (g_debouncePending &&
        ((uint32_t) (nowMs - g_edgeTimeMs) >= KEY_DEBOUNCE_TIME_MS)) {
        bool pressed = KEY_ReadRawPressed();

        if (pressed != g_stablePressed) {
            g_stablePressed = pressed;

            if (pressed) {
                g_pressedEvent      = true;
                g_longPressReported = false;
                g_pressStartMs      = nowMs;
            } else {
                uint32_t pressTimeMs = (uint32_t) (nowMs - g_pressStartMs);

                g_releasedEvent = true;
                if (!g_longPressReported) {
                    if (pressTimeMs >= KEY_LONG_PRESS_TIME_MS) {
                        g_longPressEvent = true;
                    } else {
                        g_shortPressEvent = true;
                    }
                }
            }
        }

        DL_GPIO_clearInterruptStatus(GPIO_GRP_KEY_PORT, GPIO_GRP_KEY_KEY_USER_PIN);
        g_debouncePending = false;
        DL_GPIO_enableInterrupt(GPIO_GRP_KEY_PORT, GPIO_GRP_KEY_KEY_USER_PIN);
    }

    if (g_stablePressed && !g_longPressReported &&
        ((uint32_t) (nowMs - g_pressStartMs) >= KEY_LONG_PRESS_TIME_MS)) {
        g_longPressReported = true;
        g_longPressEvent    = true;
    }
}

bool KEY_IsPressed(void)
{
    return g_stablePressed;
}

bool KEY_WasPressed(void)
{
    bool event = g_pressedEvent;

    g_pressedEvent = false;
    return event;
}

bool KEY_WasReleased(void)
{
    bool event = g_releasedEvent;

    g_releasedEvent = false;
    return event;
}

bool KEY_WasShortPressed(void)
{
    bool event = g_shortPressEvent;

    g_shortPressEvent = false;
    return event;
}

bool KEY_WasLongPressed(void)
{
    bool event = g_longPressEvent;

    g_longPressEvent = false;
    return event;
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case GPIO_GRP_KEY_INT_IIDX:
            DL_GPIO_disableInterrupt(GPIO_GRP_KEY_PORT, GPIO_GRP_KEY_KEY_USER_PIN);
            g_edgeTimeMs      = BSP_GetTickMs();
            g_debouncePending = true;
            break;
        default:
            break;
    }
}
