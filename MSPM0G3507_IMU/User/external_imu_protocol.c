#include "external_imu_protocol.h"

#define EXTERNAL_IMU_PROTOCOL_NATIVE_MIN_LEN (2U)

static int16_t ExternalIMUProtocol_ClampInt16(int32_t value)
{
    if (value > 32767) {
        return 32767;
    }

    if (value < -32768) {
        return -32768;
    }

    return (int16_t)value;
}

static int32_t ExternalIMUProtocol_DivideRoundNearest(
    int64_t numerator, int32_t denominator)
{
    if (numerator >= 0) {
        return (int32_t)((numerator + (denominator / 2)) / denominator);
    }

    return (int32_t)((numerator - (denominator / 2)) / denominator);
}

static void ExternalIMUProtocol_WriteU16LE(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void ExternalIMUProtocol_WriteI16LE(uint8_t *out, int16_t value)
{
    ExternalIMUProtocol_WriteU16LE(out, (uint16_t)value);
}

static void ExternalIMUProtocol_WriteU32LE(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value & 0xFFU);
    out[1] = (uint8_t)((value >> 8) & 0xFFU);
    out[2] = (uint8_t)((value >> 16) & 0xFFU);
    out[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static void ExternalIMUProtocol_WriteI32LE(uint8_t *out, int32_t value)
{
    ExternalIMUProtocol_WriteU32LE(out, (uint32_t)value);
}

static uint8_t ExternalIMUProtocol_Sum8(
    const uint8_t *data, size_t length)
{
    uint8_t sum = 0U;

    for (size_t index = 0U; index < length; index++) {
        sum = (uint8_t)(sum + data[index]);
    }

    return sum;
}

static void ExternalIMUProtocol_ResetParser(
    ExternalIMUProtocol_Parser *parser)
{
    parser->state     = EXTERNAL_IMU_PARSE_SYNC_0;
    parser->length    = 0U;
    parser->bodyIndex = 0U;
    parser->crcLow    = 0U;
}

uint16_t ExternalIMUProtocol_Crc16Ccitt(
    const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;

    if ((data == NULL) && (length != 0U)) {
        return 0U;
    }

    for (size_t index = 0U; index < length; index++) {
        crc ^= (uint16_t)data[index] << 8;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 0x8000U) != 0U)
                      ? (uint16_t)((crc << 1) ^ 0x1021U)
                      : (uint16_t)(crc << 1);
        }
    }

    return crc;
}

void ExternalIMUProtocol_InitParser(ExternalIMUProtocol_Parser *parser)
{
    if (parser == NULL) {
        return;
    }

    ExternalIMUProtocol_ResetParser(parser);
}

bool ExternalIMUProtocol_ParseNativeByte(ExternalIMUProtocol_Parser *parser,
    uint8_t byte, ExternalIMUProtocol_Frame *frame)
{
    uint8_t crcBytes[1U + 2U + EXTERNAL_IMU_PROTOCOL_NATIVE_MAX_PAYLOAD];
    uint16_t expectedCrc;
    uint16_t receivedCrc;

    if ((parser == NULL) || (frame == NULL)) {
        return false;
    }

    switch (parser->state) {
        case EXTERNAL_IMU_PARSE_SYNC_0:
            if (byte == EXTERNAL_IMU_PROTOCOL_NATIVE_SYNC0) {
                parser->state = EXTERNAL_IMU_PARSE_SYNC_1;
            }
            break;

        case EXTERNAL_IMU_PARSE_SYNC_1:
            if (byte == EXTERNAL_IMU_PROTOCOL_NATIVE_SYNC1) {
                parser->state = EXTERNAL_IMU_PARSE_LEN;
            } else if (byte != EXTERNAL_IMU_PROTOCOL_NATIVE_SYNC0) {
                ExternalIMUProtocol_ResetParser(parser);
            }
            break;

        case EXTERNAL_IMU_PARSE_LEN:
            if ((byte < EXTERNAL_IMU_PROTOCOL_NATIVE_MIN_LEN) ||
                (byte > (2U + EXTERNAL_IMU_PROTOCOL_NATIVE_MAX_PAYLOAD))) {
                ExternalIMUProtocol_ResetParser(parser);
            } else {
                parser->length    = byte;
                parser->bodyIndex = 0U;
                parser->state     = EXTERNAL_IMU_PARSE_BODY;
            }
            break;

        case EXTERNAL_IMU_PARSE_BODY:
            parser->body[parser->bodyIndex] = byte;
            parser->bodyIndex++;
            if (parser->bodyIndex >= parser->length) {
                parser->state = EXTERNAL_IMU_PARSE_CRC_L;
            }
            break;

        case EXTERNAL_IMU_PARSE_CRC_L:
            parser->crcLow = byte;
            parser->state  = EXTERNAL_IMU_PARSE_CRC_H;
            break;

        case EXTERNAL_IMU_PARSE_CRC_H:
            crcBytes[0] = parser->length;
            for (uint8_t index = 0U; index < parser->length; index++) {
                crcBytes[1U + index] = parser->body[index];
            }

            expectedCrc = ExternalIMUProtocol_Crc16Ccitt(
                crcBytes, (size_t)parser->length + 1U);
            receivedCrc = (uint16_t)parser->crcLow | ((uint16_t)byte << 8);
            if (expectedCrc == receivedCrc) {
                frame->type          = parser->body[0];
                frame->msgId         = parser->body[1];
                frame->payloadLength = (uint8_t)(parser->length - 2U);
                for (uint8_t index = 0U; index < frame->payloadLength;
                     index++) {
                    frame->payload[index] = parser->body[2U + index];
                }
                ExternalIMUProtocol_ResetParser(parser);
                return true;
            }
            ExternalIMUProtocol_ResetParser(parser);
            break;

        default:
            ExternalIMUProtocol_ResetParser(parser);
            break;
    }

    return false;
}

size_t ExternalIMUProtocol_EncodeNativeFrame(uint8_t frameType, uint8_t msgId,
    const uint8_t *payload, uint8_t payloadLength, uint8_t *out,
    size_t outSize)
{
    uint8_t length = (uint8_t)(2U + payloadLength);
    size_t frameLength;
    uint16_t crc;

    if ((out == NULL) || ((payload == NULL) && (payloadLength != 0U)) ||
        (payloadLength > EXTERNAL_IMU_PROTOCOL_NATIVE_MAX_PAYLOAD)) {
        return 0U;
    }

    frameLength = 2U + 1U + (size_t)length + 2U;
    if (outSize < frameLength) {
        return 0U;
    }

    out[0] = EXTERNAL_IMU_PROTOCOL_NATIVE_SYNC0;
    out[1] = EXTERNAL_IMU_PROTOCOL_NATIVE_SYNC1;
    out[2] = length;
    out[3] = frameType;
    out[4] = msgId;
    for (uint8_t index = 0U; index < payloadLength; index++) {
        out[5U + index] = payload[index];
    }

    crc = ExternalIMUProtocol_Crc16Ccitt(&out[2], (size_t)length + 1U);
    ExternalIMUProtocol_WriteU16LE(&out[frameLength - 2U], crc);

    return frameLength;
}

size_t ExternalIMUProtocol_EncodeNativeResponse(uint8_t command,
    ExternalIMUProtocol_Status status, uint8_t imuState, uint8_t calResult,
    uint8_t *out, size_t outSize)
{
    uint8_t payload[4];

    payload[0] = (uint8_t)status;
    payload[1] = imuState;
    payload[2] = calResult;
    payload[3] = 0U;

    return ExternalIMUProtocol_EncodeNativeFrame(
        EXTERNAL_IMU_FRAME_RESPONSE, command, payload, (uint8_t)sizeof(payload),
        out, outSize);
}

size_t ExternalIMUProtocol_EncodeNativeIMUData(
    const ExternalIMUProtocol_NativeDataPayload *payload, uint8_t *out,
    size_t outSize)
{
    uint8_t data[EXTERNAL_IMU_PROTOCOL_NATIVE_DATA_PAYLOAD_SIZE];
    size_t offset = 0U;

    if (payload == NULL) {
        return 0U;
    }

    ExternalIMUProtocol_WriteU32LE(&data[offset], payload->timeMs);
    offset += 4U;
    ExternalIMUProtocol_WriteI32LE(&data[offset], payload->angularRateMilliDps);
    offset += 4U;
    ExternalIMUProtocol_WriteI32LE(&data[offset], payload->angleMilliDeg);
    offset += 4U;
    ExternalIMUProtocol_WriteI32LE(
        &data[offset], payload->normalizedAngleMilliDeg);
    offset += 4U;
    data[offset] = payload->imuState;
    offset++;
    data[offset] = payload->appState;
    offset++;
    data[offset] = payload->calResult;
    offset++;
    data[offset] = payload->flags;
    offset++;

    if (offset != EXTERNAL_IMU_PROTOCOL_NATIVE_DATA_PAYLOAD_SIZE) {
        return 0U;
    }

    return ExternalIMUProtocol_EncodeNativeFrame(EXTERNAL_IMU_FRAME_DATA,
        EXTERNAL_IMU_DATA_IMU_STATUS, data, (uint8_t)sizeof(data), out,
        outSize);
}

size_t ExternalIMUProtocol_EncodeJY901GyroFrame(
    int32_t wzMilliDps, uint8_t *out, size_t outSize)
{
    int32_t raw = ExternalIMUProtocol_DivideRoundNearest(
        (int64_t)wzMilliDps * 32768LL, 2000000);

    if ((out == NULL) || (outSize < EXTERNAL_IMU_PROTOCOL_JY901_FRAME_SIZE)) {
        return 0U;
    }

    out[0] = 0x55U;
    out[1] = 0x52U;
    ExternalIMUProtocol_WriteI16LE(&out[2], 0);
    ExternalIMUProtocol_WriteI16LE(&out[4], 0);
    ExternalIMUProtocol_WriteI16LE(
        &out[6], ExternalIMUProtocol_ClampInt16(raw));
    ExternalIMUProtocol_WriteI16LE(&out[8], 0);
    out[10] = ExternalIMUProtocol_Sum8(out, 10U);

    return EXTERNAL_IMU_PROTOCOL_JY901_FRAME_SIZE;
}

size_t ExternalIMUProtocol_EncodeJY901AngleFrame(
    int32_t yawMilliDeg, uint8_t *out, size_t outSize)
{
    int32_t raw = ExternalIMUProtocol_DivideRoundNearest(
        (int64_t)yawMilliDeg * 32768LL, 180000);

    if ((out == NULL) || (outSize < EXTERNAL_IMU_PROTOCOL_JY901_FRAME_SIZE)) {
        return 0U;
    }

    out[0] = 0x55U;
    out[1] = 0x53U;
    ExternalIMUProtocol_WriteI16LE(&out[2], 0);
    ExternalIMUProtocol_WriteI16LE(&out[4], 0);
    ExternalIMUProtocol_WriteI16LE(
        &out[6], ExternalIMUProtocol_ClampInt16(raw));
    ExternalIMUProtocol_WriteI16LE(&out[8], 0);
    out[10] = ExternalIMUProtocol_Sum8(out, 10U);

    return EXTERNAL_IMU_PROTOCOL_JY901_FRAME_SIZE;
}
