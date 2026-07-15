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
 *   HEATER,STATE,<0 or 1>,<duty percent with one decimal place>
 *   HEATER,CTRL,<time ms>,<phase>,<temperature mC>,<target mC>,
 *       <duty permille>,<feedforward permille>,<PID correction permille>,
 *       <Kp x1000>,<Ki x1000>,<Kd x1000>
 *
 * Host commands on Type-C UART:
 *   START / HELLO / ON  enable continuous reporting
 *   STOP / OFF          disable continuous reporting
 *   PING                keep reporting alive before timeout
 */
typedef enum {
    HOST_LINK_HEATER_PHASE_RAPID = 1,
    HOST_LINK_HEATER_PHASE_PID = 2,
} HostLink_HeaterPhase;

typedef struct {
    uint32_t timeMs;
    HostLink_HeaterPhase phase;
    int32_t temperatureMilliC;
    int32_t targetMilliC;
    uint16_t dutyPermille;
    uint16_t feedforwardPermille;
    int32_t pidCorrectionPermille;
    int32_t kpMilli;
    int32_t kiMilli;
    int32_t kdMilli;
} HostLink_HeaterControlSample;

void HostLink_Init(void);
void HostLink_Run(void);
bool HostLink_IsReportingEnabled(void);

bool HostLink_SendTMP117TemperatureRaw(int16_t rawTemperature);
bool HostLink_SendTMP117Error(TMP117_Status status);
bool HostLink_SendHeaterState(bool enabled, uint16_t dutyPermille);
bool HostLink_SendHeaterControlSample(
    const HostLink_HeaterControlSample *sample);

#endif
