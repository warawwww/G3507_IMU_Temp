#ifndef APP_H
#define APP_H

#include <stdbool.h>

#include "TMP117.h"

void APP_Init(void);
void APP_Run(void);

bool APP_GetTemperatureC(float *temperatureC);
TMP117_Status APP_GetTemperatureStatus(void);

#endif
