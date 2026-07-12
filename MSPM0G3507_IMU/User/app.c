#include "app.h"

#include "KEY.h"

void APP_Init(void)
{
    KEY_Init();
}

void APP_Run(void)
{
    KEY_Update();
}
