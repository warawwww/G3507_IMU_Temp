#include "LED.h"

#include <stdint.h>

#include "ti_msp_dl_config.h"

static uint32_t LED_GetPin(LED_ID led)
{
    switch (led) {
        case LED_ID_GREEN:
            return GPIO_GRP_LEDS_LED_GRN_PIN;
        case LED_ID_RED:
            return GPIO_GRP_LEDS_LED_RED_PIN;
        default:
            return 0U;
    }
}

void LED_Init(void)
{
    LED_Off(LED_ID_GREEN);
    LED_Off(LED_ID_RED);
}

void LED_Set(LED_ID led, bool on)
{
    uint32_t pin = LED_GetPin(led);

    if (pin == 0U) {
        return;
    }

    /* 板载 LED 低电平点亮。 */
    if (on) {
        DL_GPIO_clearPins(GPIO_GRP_LEDS_PORT, pin);
    } else {
        DL_GPIO_setPins(GPIO_GRP_LEDS_PORT, pin);
    }
}

void LED_On(LED_ID led)
{
    LED_Set(led, true);
}

void LED_Off(LED_ID led)
{
    LED_Set(led, false);
}

void LED_Toggle(LED_ID led)
{
    uint32_t pin = LED_GetPin(led);

    if (pin == 0U) {
        return;
    }

    DL_GPIO_togglePins(GPIO_GRP_LEDS_PORT, pin);
}
