#include "external_imu_link.h"

#include <stddef.h>
#include <stdint.h>

#include "app_config.h"
#include "bsp.h"
#include "bsp_uart.h"
#include "external_imu_protocol.h"
#include "imu_task.h"

#define EXTERNAL_IMU_LINK_NATIVE_PERIOD_MS (10U)
#define EXTERNAL_IMU_LINK_JY901_PERIOD_MS  (100U)

#define EXTERNAL_IMU_LINK_FLAG_STATIC_BIAS_LEARNING (0x01U)
#define EXTERNAL_IMU_LINK_FLAG_RX_OVERFLOW          (0x02U)

static ExternalIMUProtocol_Parser g_parser;
static bool g_zeroCalibrationRequested;
static bool g_autoCRequested;
static bool g_rotationCalibrationRequested;
static bool g_angleResetRequested;
static uint32_t g_lastNativeTxMs;
static uint32_t g_lastJY901TxMs;
static uint8_t g_pendingResponse[EXTERNAL_IMU_PROTOCOL_NATIVE_MAX_FRAME_SIZE];
static size_t g_pendingResponseLength;

static int32_t ExternalIMULink_RoundScaled(float value, float scale)
{
    float scaled = value * scale;

    if (scaled >= 0.0f) {
        return (int32_t)(scaled + 0.5f);
    }

    return (int32_t)(scaled - 0.5f);
}

static bool ExternalIMULink_TakeFlag(bool *flag)
{
    bool result = *flag;

    *flag = false;
    return result;
}

static bool ExternalIMULink_WriteFrame(
    const uint8_t *frame, size_t frameLength)
{
    if ((frame == NULL) || (frameLength == 0U) ||
        BSP_UART_IsTxBusy(BSP_UART_PORT_EXTERNAL)) {
        return false;
    }

    return BSP_UART_Write(BSP_UART_PORT_EXTERNAL, frame, frameLength);
}

static void ExternalIMULink_QueueResponse(
    const uint8_t *frame, size_t frameLength)
{
    if ((frame == NULL) || (frameLength == 0U) ||
        (frameLength > sizeof(g_pendingResponse))) {
        return;
    }

    if (g_pendingResponseLength != 0U) {
        return;
    }

    for (size_t index = 0U; index < frameLength; index++) {
        g_pendingResponse[index] = frame[index];
    }
    g_pendingResponseLength = frameLength;
}

static void ExternalIMULink_TrySendPendingResponse(void)
{
    if (g_pendingResponseLength == 0U) {
        return;
    }

    if (ExternalIMULink_WriteFrame(
            g_pendingResponse, g_pendingResponseLength)) {
        g_pendingResponseLength = 0U;
    }
}

static void ExternalIMULink_SendResponse(
    uint8_t command, ExternalIMUProtocol_Status status)
{
    uint8_t frame[EXTERNAL_IMU_PROTOCOL_NATIVE_MAX_FRAME_SIZE];
    size_t frameLength = ExternalIMUProtocol_EncodeNativeResponse(command,
        status, (uint8_t)IMU_Task_GetState(),
        (uint8_t)IMU_Task_GetCalibrationResult(), frame, sizeof(frame));

    if (frameLength == 0U) {
        return;
    }

    if (!ExternalIMULink_WriteFrame(frame, frameLength)) {
        ExternalIMULink_QueueResponse(frame, frameLength);
    }
}

static bool ExternalIMULink_IsPayloadEmpty(
    const ExternalIMUProtocol_Frame *frame)
{
    return frame->payloadLength == 0U;
}

static void ExternalIMULink_HandleCommand(
    const ExternalIMUProtocol_Frame *frame)
{
    ExternalIMUProtocol_Status status = EXTERNAL_IMU_STATUS_OK;
    bool calibrationBusy             = IMU_Task_IsCalibrationBusy();
    bool sensorOk = IMU_Task_GetSensorStatus() == XV7021_STATUS_OK;

    if (frame->type != EXTERNAL_IMU_FRAME_COMMAND) {
        ExternalIMULink_SendResponse(
            frame->msgId, EXTERNAL_IMU_STATUS_BAD_FRAME);
        return;
    }

    if (!ExternalIMULink_IsPayloadEmpty(frame)) {
        ExternalIMULink_SendResponse(
            frame->msgId, EXTERNAL_IMU_STATUS_BAD_PARAM);
        return;
    }

    switch (frame->msgId) {
        case EXTERNAL_IMU_CMD_PING:
            break;

        case EXTERNAL_IMU_CMD_ZERO_CAL:
            if (!sensorOk) {
                status = EXTERNAL_IMU_STATUS_FAILED;
            } else if (calibrationBusy) {
                status = EXTERNAL_IMU_STATUS_BUSY;
            } else {
                g_zeroCalibrationRequested = true;
            }
            break;

        case EXTERNAL_IMU_CMD_AUTOC:
            if (!sensorOk) {
                status = EXTERNAL_IMU_STATUS_FAILED;
            } else if (calibrationBusy) {
                status = EXTERNAL_IMU_STATUS_BUSY;
            } else {
                g_autoCRequested = true;
            }
            break;

        case EXTERNAL_IMU_CMD_CAL1080:
            if (!sensorOk) {
                status = EXTERNAL_IMU_STATUS_FAILED;
            } else if (calibrationBusy) {
                status = EXTERNAL_IMU_STATUS_BUSY;
            } else {
                g_rotationCalibrationRequested = true;
            }
            break;

        case EXTERNAL_IMU_CMD_ANGLE_ZERO:
            if (!sensorOk) {
                status = EXTERNAL_IMU_STATUS_FAILED;
            } else if (calibrationBusy) {
                status = EXTERNAL_IMU_STATUS_BUSY;
            } else {
                g_angleResetRequested = true;
            }
            break;

        default:
            status = EXTERNAL_IMU_STATUS_BAD_COMMAND;
            break;
    }

    ExternalIMULink_SendResponse(frame->msgId, status);
}

static void ExternalIMULink_ProcessRx(void)
{
#if APP_EXTERNAL_IMU_ENABLE_NATIVE_LINK
    uint8_t byte;
    ExternalIMUProtocol_Frame frame;

    while (BSP_UART_TryReadByte(BSP_UART_PORT_EXTERNAL, &byte)) {
        if (ExternalIMUProtocol_ParseNativeByte(&g_parser, byte, &frame)) {
            ExternalIMULink_HandleCommand(&frame);
        }
    }
#else
    BSP_UART_FlushRx(BSP_UART_PORT_EXTERNAL);
#endif
}

static uint8_t ExternalIMULink_GetFlags(void)
{
    uint8_t flags = 0U;

    if (IMU_Task_IsStaticBiasLearning()) {
        flags |= EXTERNAL_IMU_LINK_FLAG_STATIC_BIAS_LEARNING;
    }

    if (BSP_UART_RxOverflowed(BSP_UART_PORT_EXTERNAL)) {
        flags |= EXTERNAL_IMU_LINK_FLAG_RX_OVERFLOW;
        BSP_UART_ClearRxOverflow(BSP_UART_PORT_EXTERNAL);
    }

    return flags;
}

static void ExternalIMULink_SendNativeDataIfDue(
    uint32_t nowMs, uint8_t appState)
{
#if APP_EXTERNAL_IMU_ENABLE_NATIVE_LINK
    IMU_Task_Sample sample;
    ExternalIMUProtocol_NativeDataPayload payload;
    uint8_t frame[EXTERNAL_IMU_PROTOCOL_NATIVE_MAX_FRAME_SIZE];
    size_t frameLength;

    if ((uint32_t)(nowMs - g_lastNativeTxMs) <
        EXTERNAL_IMU_LINK_NATIVE_PERIOD_MS) {
        return;
    }

    if (!IMU_Task_GetSample(&sample)) {
        return;
    }

    payload.timeMs = sample.timeMs;
    payload.angularRateMilliDps =
        ExternalIMULink_RoundScaled(sample.angularRateDps, 1000.0f);
    payload.angleMilliDeg =
        ExternalIMULink_RoundScaled(sample.angleDeg, 1000.0f);
    payload.normalizedAngleMilliDeg =
        ExternalIMULink_RoundScaled(sample.normalizedAngleDeg, 1000.0f);
    payload.imuState  = (uint8_t)sample.state;
    payload.appState  = appState;
    payload.calResult = (uint8_t)IMU_Task_GetCalibrationResult();
    payload.flags     = ExternalIMULink_GetFlags();

    frameLength = ExternalIMUProtocol_EncodeNativeIMUData(
        &payload, frame, sizeof(frame));
    if ((frameLength != 0U) &&
        ExternalIMULink_WriteFrame(frame, frameLength)) {
        g_lastNativeTxMs = nowMs;
    }
#else
    (void)nowMs;
    (void)appState;
#endif
}

static void ExternalIMULink_SendJY901DataIfDue(uint32_t nowMs)
{
#if APP_EXTERNAL_IMU_ENABLE_JY901_STREAM
    IMU_Task_Sample sample;
    uint8_t frame[EXTERNAL_IMU_PROTOCOL_JY901_FRAME_SIZE * 2U];
    int32_t angularRateMilliDps;
    int32_t normalizedAngleMilliDeg;
    size_t firstLength;
    size_t secondLength;

    if ((uint32_t)(nowMs - g_lastJY901TxMs) <
        EXTERNAL_IMU_LINK_JY901_PERIOD_MS) {
        return;
    }

    if (!IMU_Task_GetSample(&sample)) {
        return;
    }

    angularRateMilliDps =
        ExternalIMULink_RoundScaled(sample.angularRateDps, 1000.0f);
    normalizedAngleMilliDeg =
        ExternalIMULink_RoundScaled(sample.normalizedAngleDeg, 1000.0f);

    firstLength = ExternalIMUProtocol_EncodeJY901GyroFrame(
        angularRateMilliDps, frame, sizeof(frame));
    secondLength = ExternalIMUProtocol_EncodeJY901AngleFrame(
        normalizedAngleMilliDeg, &frame[firstLength],
        sizeof(frame) - firstLength);

    if ((firstLength == EXTERNAL_IMU_PROTOCOL_JY901_FRAME_SIZE) &&
        (secondLength == EXTERNAL_IMU_PROTOCOL_JY901_FRAME_SIZE) &&
        ExternalIMULink_WriteFrame(frame, firstLength + secondLength)) {
        g_lastJY901TxMs = nowMs;
    }
#else
    (void)nowMs;
#endif
}

void ExternalIMULink_Init(void)
{
    uint32_t nowMs = BSP_GetTickMs();

    ExternalIMUProtocol_InitParser(&g_parser);
    BSP_UART_FlushRx(BSP_UART_PORT_EXTERNAL);

    g_zeroCalibrationRequested     = false;
    g_autoCRequested               = false;
    g_rotationCalibrationRequested = false;
    g_angleResetRequested          = false;
    g_lastNativeTxMs               = nowMs;
    g_lastJY901TxMs                = nowMs;
    g_pendingResponseLength        = 0U;
}

void ExternalIMULink_Run(uint8_t appState)
{
    uint32_t nowMs = BSP_GetTickMs();

    ExternalIMULink_TrySendPendingResponse();
    ExternalIMULink_ProcessRx();

    if (g_pendingResponseLength != 0U) {
        return;
    }

    ExternalIMULink_SendNativeDataIfDue(nowMs, appState);
    ExternalIMULink_SendJY901DataIfDue(nowMs);
}

bool ExternalIMULink_TakeZeroCalibrationRequest(void)
{
    return ExternalIMULink_TakeFlag(&g_zeroCalibrationRequested);
}

bool ExternalIMULink_TakeAutoCRequest(void)
{
    return ExternalIMULink_TakeFlag(&g_autoCRequested);
}

bool ExternalIMULink_TakeRotationCalibrationRequest(void)
{
    return ExternalIMULink_TakeFlag(&g_rotationCalibrationRequested);
}

bool ExternalIMULink_TakeAngleResetRequest(void)
{
    return ExternalIMULink_TakeFlag(&g_angleResetRequested);
}
