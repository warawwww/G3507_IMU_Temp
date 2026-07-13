#ifndef USER_HOST_LINK_H
#define USER_HOST_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "TMP117.h"

/*
 * Application protocol between the device and the host computer.
 * Each Type-C UART message is one ASCII frame per line. The first field
 * identifies the data source so more devices can be added later.
 * Current TMP117 frames:
 *   TMP117,T,<temperature in milli-degrees Celsius>
 *   TMP117,ERR,<TMP117_Status value>
 * Current heater frame:
 *   HEATER,STATE,<0 or 1>,<duty percent>
 */
bool HostLink_SendTMP117TemperatureRaw(int16_t rawTemperature);
bool HostLink_SendTMP117Error(TMP117_Status status);
bool HostLink_SendHeaterState(bool enabled, uint8_t dutyPercent);

#endif
