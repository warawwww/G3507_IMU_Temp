#include "app.h"

#include "KEY.h"
#include "LED.h"
#include "TMP117.h"

void APP_Init(void)
{
    LED_Init();
    KEY_Init();
    (void) TMP117_Init();
}

void APP_Run(void)
{
    KEY_Update();
}
