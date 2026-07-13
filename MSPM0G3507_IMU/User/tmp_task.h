#ifndef USER_TMP_TASK_H
#define USER_TMP_TASK_H

#include <stdbool.h>

#include "TMP117.h"

void TMP_Task_Init(void);
void TMP_Task_Run(void);

/* Returns false until a valid temperature sample is available. */
bool TMP_Task_GetTemperatureC(float *temperatureC);
TMP117_Status TMP_Task_GetStatus(void);

#endif
