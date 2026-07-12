#include "app.h"
#include "bsp.h"

int main(void)
{
    BSP_Init();
    APP_Init();

    while (1) {
        APP_Run();
    }
}
