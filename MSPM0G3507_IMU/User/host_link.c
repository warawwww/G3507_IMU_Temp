#include "host_link.h"

#include <stddef.h>
#include <string.h>

#include "bsp.h"
#include "bsp_uart.h"

#define HOST_LINK_FRAME_CAPACITY (96U)
#define HOST_LINK_RX_LINE_CAPACITY (32U)
#define HOST_LINK_REPORT_TIMEOUT_MS (5000U)

static bool g_reportingEnabled;
static uint32_t g_lastHostMessageMs;
static char g_rxLine[HOST_LINK_RX_LINE_CAPACITY];
static size_t g_rxLineLength;

static bool HostLink_WriteRawString(const char *string)
{
    size_t length;

    if (string == NULL) {
        return false;
    }

    length = strlen(string);
    return BSP_UART_Write(
        BSP_UART_PORT_TYPEC, (const uint8_t *) string, length);
}

static void HostLink_EnableReporting(uint32_t nowMs)
{
    g_reportingEnabled = true;
    g_lastHostMessageMs = nowMs;
    (void) HostLink_WriteRawString("HOST,OK,START\r\n");
}

static void HostLink_DisableReporting(uint32_t nowMs)
{
    g_reportingEnabled = false;
    g_lastHostMessageMs = nowMs;
    (void) HostLink_WriteRawString("HOST,OK,STOP\r\n");
}

static void HostLink_ProcessCommand(const char *line)
{
    uint32_t nowMs = BSP_GetTickMs();

    if ((strcmp(line, "START") == 0) ||
        (strcmp(line, "HELLO") == 0) ||
        (strcmp(line, "ON") == 0)) {
        HostLink_EnableReporting(nowMs);
    } else if ((strcmp(line, "STOP") == 0) ||
        (strcmp(line, "OFF") == 0)) {
        HostLink_DisableReporting(nowMs);
    } else if (strcmp(line, "PING") == 0) {
        g_lastHostMessageMs = nowMs;
        (void) HostLink_WriteRawString(
            g_reportingEnabled ? "HOST,PONG,1\r\n" : "HOST,PONG,0\r\n");
    } else {
        (void) HostLink_WriteRawString("HOST,ERR,CMD\r\n");
    }
}

static void HostLink_ProcessRxByte(uint8_t byte)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        g_rxLine[g_rxLineLength] = '\0';
        if (g_rxLineLength != 0U) {
            HostLink_ProcessCommand(g_rxLine);
        }
        g_rxLineLength = 0U;
        return;
    }

    if (g_rxLineLength >= (sizeof(g_rxLine) - 1U)) {
        g_rxLineLength = 0U;
        return;
    }

    g_rxLine[g_rxLineLength++] = (char) byte;
}

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

    if (!g_reportingEnabled) {
        return true;
    }

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

    if (!g_reportingEnabled) {
        return true;
    }

    return HostLink_SendNumberFrame(
        prefix, sizeof(prefix) - 1U, (int32_t) status);
}

bool HostLink_SendHeaterState(bool enabled, uint16_t dutyPermille)
{
    static const char prefix[] = "HEATER,STATE,";
    char frame[HOST_LINK_FRAME_CAPACITY];
    size_t length;

    if (!g_reportingEnabled) {
        return true;
    }

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

    if (!g_reportingEnabled) {
        return true;
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

void HostLink_Init(void)
{
    g_reportingEnabled = false;
    g_lastHostMessageMs = BSP_GetTickMs();
    g_rxLineLength = 0U;
    BSP_UART_FlushRx(BSP_UART_PORT_TYPEC);
}

void HostLink_Run(void)
{
    uint8_t byte;
    uint32_t nowMs = BSP_GetTickMs();

    while (BSP_UART_TryReadByte(BSP_UART_PORT_TYPEC, &byte)) {
        HostLink_ProcessRxByte(byte);
    }

    if (g_reportingEnabled &&
        ((uint32_t) (nowMs - g_lastHostMessageMs) >=
            HOST_LINK_REPORT_TIMEOUT_MS)) {
        g_reportingEnabled = false;
    }
}

bool HostLink_IsReportingEnabled(void)
{
    return g_reportingEnabled;
}
