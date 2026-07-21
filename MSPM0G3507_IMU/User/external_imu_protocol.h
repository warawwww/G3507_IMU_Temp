#ifndef USER_EXTERNAL_IMU_PROTOCOL_H
#define USER_EXTERNAL_IMU_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define EXTERNAL_IMU_PROTOCOL_NATIVE_SYNC0          (0xA5U)
#define EXTERNAL_IMU_PROTOCOL_NATIVE_SYNC1          (0x5AU)
#define EXTERNAL_IMU_PROTOCOL_NATIVE_MAX_PAYLOAD    (64U)
#define EXTERNAL_IMU_PROTOCOL_NATIVE_MAX_FRAME_SIZE (71U)
#define EXTERNAL_IMU_PROTOCOL_JY901_FRAME_SIZE      (11U)
#define EXTERNAL_IMU_PROTOCOL_NATIVE_DATA_PAYLOAD_SIZE \
    (20U)

typedef enum {
    EXTERNAL_IMU_FRAME_COMMAND  = 0x01U,
    EXTERNAL_IMU_FRAME_RESPONSE = 0x02U,
    EXTERNAL_IMU_FRAME_DATA     = 0x03U,
} ExternalIMUProtocol_FrameType;

typedef enum {
    EXTERNAL_IMU_CMD_PING       = 0x01U,
    EXTERNAL_IMU_CMD_ZERO_CAL   = 0x10U,
    EXTERNAL_IMU_CMD_AUTOC      = 0x11U,
    EXTERNAL_IMU_CMD_CAL1080    = 0x12U,
    EXTERNAL_IMU_CMD_ANGLE_ZERO = 0x13U,
} ExternalIMUProtocol_Command;

typedef enum {
    EXTERNAL_IMU_STATUS_OK          = 0x00U,
    EXTERNAL_IMU_STATUS_BAD_CRC     = 0x01U,
    EXTERNAL_IMU_STATUS_BAD_FRAME   = 0x02U,
    EXTERNAL_IMU_STATUS_BAD_COMMAND = 0x03U,
    EXTERNAL_IMU_STATUS_BAD_PARAM   = 0x04U,
    EXTERNAL_IMU_STATUS_BUSY        = 0x05U,
    EXTERNAL_IMU_STATUS_FAILED      = 0x06U,
} ExternalIMUProtocol_Status;

typedef enum {
    EXTERNAL_IMU_DATA_IMU_STATUS = 0x01U,
} ExternalIMUProtocol_DataId;

typedef enum {
    EXTERNAL_IMU_DATA_FLAG_STATIC_BIAS_LEARNING = 0x01U,
    EXTERNAL_IMU_DATA_FLAG_RX_OVERFLOW          = 0x02U,
    EXTERNAL_IMU_DATA_FLAG_ANGLE_VALID          = 0x04U,
} ExternalIMUProtocol_DataFlag;

typedef enum {
    EXTERNAL_IMU_PARSE_SYNC_0 = 0,
    EXTERNAL_IMU_PARSE_SYNC_1,
    EXTERNAL_IMU_PARSE_LEN,
    EXTERNAL_IMU_PARSE_BODY,
    EXTERNAL_IMU_PARSE_CRC_L,
    EXTERNAL_IMU_PARSE_CRC_H,
} ExternalIMUProtocol_ParseState;

typedef struct {
    uint8_t type;
    uint8_t msgId;
    uint8_t payloadLength;
    uint8_t payload[EXTERNAL_IMU_PROTOCOL_NATIVE_MAX_PAYLOAD];
} ExternalIMUProtocol_Frame;

typedef struct {
    ExternalIMUProtocol_ParseState state;
    uint8_t length;
    uint8_t bodyIndex;
    uint8_t body[2U + EXTERNAL_IMU_PROTOCOL_NATIVE_MAX_PAYLOAD];
    uint8_t crcLow;
} ExternalIMUProtocol_Parser;

typedef struct {
    uint32_t timeMs;
    int32_t angularRateMilliDps;
    int32_t angleMilliDeg;
    int32_t normalizedAngleMilliDeg;
    uint8_t imuState;
    uint8_t appState;
    uint8_t calResult;
    uint8_t flags;
} ExternalIMUProtocol_NativeDataPayload;

uint16_t ExternalIMUProtocol_Crc16Ccitt(const uint8_t *data, size_t length);
int32_t ExternalIMUProtocol_NormalizeAngleMilliDeg180(
    int32_t angleMilliDeg);

void ExternalIMUProtocol_InitParser(ExternalIMUProtocol_Parser *parser);
bool ExternalIMUProtocol_ParseNativeByte(ExternalIMUProtocol_Parser *parser,
    uint8_t byte, ExternalIMUProtocol_Frame *frame);

size_t ExternalIMUProtocol_EncodeNativeFrame(uint8_t frameType, uint8_t msgId,
    const uint8_t *payload, uint8_t payloadLength, uint8_t *out,
    size_t outSize);
size_t ExternalIMUProtocol_EncodeNativeResponse(uint8_t command,
    ExternalIMUProtocol_Status status, uint8_t imuState, uint8_t calResult,
    uint8_t *out, size_t outSize);
size_t ExternalIMUProtocol_EncodeNativeIMUData(
    const ExternalIMUProtocol_NativeDataPayload *payload, uint8_t *out,
    size_t outSize);

size_t ExternalIMUProtocol_EncodeJY901GyroFrame(
    int32_t wzMilliDps, uint8_t *out, size_t outSize);
size_t ExternalIMUProtocol_EncodeJY901AngleFrame(
    int32_t yawMilliDeg, uint8_t *out, size_t outSize);

#endif
