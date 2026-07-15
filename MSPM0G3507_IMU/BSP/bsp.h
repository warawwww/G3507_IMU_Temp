#ifndef BSP_H
#define BSP_H

#include <stdint.h>

void BSP_Init(void);
uint32_t BSP_GetTickMs(void);
uint32_t BSP_GetTickUs(void);

#endif
