#include "host_link.h"

#include <stddef.h>
#include <string.h>

#include "bsp_uart.h"

#define HOST_LINK_FRAME_CAPACITY (96U)

static size_t HostLink_AppendInt32(char *buffer, size_t position,
    int32_t value)
{
    char reversedDigits[10];
    uint32_t magnitude;
    size_t digitCount = 0U;

    if (value < 0) {
        buffer[position++] = '-';
        magnitude = (uint32_t) (-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t) value;
    }

    do {
        reversedDigits[digitCount++] = (char) ('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while (magnitude != 0U);

    while (digitCount != 0U) {
        buffer[position++] = reversedDigits[--digitCount];
    }

    return position;
}

static bool HostLink_SendNumberFrame(
    const char *prefix, size_t prefixLength, int32_t value)
{
    char frame[HOST_LINK_FRAME_CAPACITY];
    size_t length;

    if (prefixLength > (sizeof(frame) - 14U)) {
        return false;
    }

    memcpy(frame, prefix, prefixLength);
    length          = HostLink_AppendInt32(frame, prefixLength, value);
    frame[length++] = '\r';
    frame[length++] = '\n';

    return BSP_UART_Write(
        BSP_UART_PORT_TYPEC, (const uint8_t *) frame, length);
}

bool HostLink_SendTMP117TemperatureRaw(int16_t rawTemperature)
{
    static const char prefix[] = "TMP117,T,";
    int32_t scaledTemperature = (int32_t) rawTemperature * 1000;
    int32_t temperatureMilliC;

    /* TMP117 temperature LSB is exactly 1 / 128 degree Celsius. */
    if (scaledTemperature >= 0) {
        scaledTemperature += 64;
    } else {
        scaledTemperature -= 64;
    }
    temperatureMilliC = scaledTemperature / 128;

    return HostLink_SendNumberFrame(
        prefix, sizeof(prefix) - 1U, temperatureMilliC);
}

bool HostLink_SendTMP117Error(TMP117_Status status)
{
    static const char prefix[] = "TMP117,ERR,";

    return HostLink_SendNumberFrame(
        prefix, sizeof(prefix) - 1U, (int32_t) status);
}

bool HostLink_SendHeaterState(bool enabled, uint16_t dutyPermille)
{
    static const char prefix[] = "HEATER,STATE,";
    char frame[HOST_LINK_FRAME_CAPACITY];
    size_t length;

    memcpy(frame, prefix, sizeof(prefix) - 1U);
    length = sizeof(prefix) - 1U;
    frame[length++] = enabled ? '1' : '0';
    frame[length++] = ',';
    length = HostLink_AppendInt32(
        frame, length, (int32_t) (dutyPermille / 10U));
    frame[length++] = '.';
    frame[length++] = (char) ('0' + (dutyPermille % 10U));
    frame[length++] = '\r';
    frame[length++] = '\n';

    return BSP_UART_Write(
        BSP_UART_PORT_TYPEC, (const uint8_t *) frame, length);
}

bool HostLink_SendHeaterControlSample(
    const HostLink_HeaterControlSample *sample)
{
    static const char prefix[] = "HEATER,CTRL,";
    int32_t fields[10];
    char frame[HOST_LINK_FRAME_CAPACITY];
    size_t length;
    size_t index;

    if (sample == NULL) {
        return false;
    }

    fields[0] = (int32_t) sample->timeMs;
    fields[1] = (int32_t) sample->phase;
    fields[2] = sample->temperatureMilliC;
    fields[3] = sample->targetMilliC;
    fields[4] = (int32_t) sample->dutyPermille;
    fields[5] = (int32_t) sample->feedforwardPermille;
    fields[6] = sample->pidCorrectionPermille;
    fields[7] = sample->kpMilli;
    fields[8] = sample->kiMilli;
    fields[9] = sample->kdMilli;

    memcpy(frame, prefix, sizeof(prefix) - 1U);
    length = sizeof(prefix) - 1U;
    for (index = 0U; index < (sizeof(fields) / sizeof(fields[0])); index++) {
        length = HostLink_AppendInt32(frame, length, fields[index]);
        frame[length++] = (index + 1U ==
            (sizeof(fields) / sizeof(fields[0]))) ? '\r' : ',';
    }
    frame[length++] = '\n';

    if (length > sizeof(frame)) {
        return false;
    }

    return BSP_UART_Write(
        BSP_UART_PORT_TYPEC, (const uint8_t *) frame, length);
}
