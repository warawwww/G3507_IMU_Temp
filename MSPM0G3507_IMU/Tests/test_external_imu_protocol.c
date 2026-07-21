#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include "external_imu_protocol.h"

static void test_crc16_ccitt_reference(void)
{
    static const uint8_t data[] = {
        '1', '2', '3', '4', '5', '6', '7', '8', '9'};

    assert(ExternalIMUProtocol_Crc16Ccitt(data, sizeof(data)) == 0x29B1U);
}

static void test_native_command_round_trip(void)
{
    uint8_t bytes[EXTERNAL_IMU_PROTOCOL_NATIVE_MAX_FRAME_SIZE];
    ExternalIMUProtocol_Parser parser;
    ExternalIMUProtocol_Frame parsed;
    size_t length;
    bool completed = false;

    length = ExternalIMUProtocol_EncodeNativeFrame(
        EXTERNAL_IMU_FRAME_COMMAND, EXTERNAL_IMU_CMD_PING, NULL, 0U, bytes,
        sizeof(bytes));
    assert(length == 7U);

    ExternalIMUProtocol_InitParser(&parser);
    for (size_t index = 0U; index < length; index++) {
        completed = ExternalIMUProtocol_ParseNativeByte(
            &parser, bytes[index], &parsed);
    }

    assert(completed);
    assert(parsed.type == EXTERNAL_IMU_FRAME_COMMAND);
    assert(parsed.msgId == EXTERNAL_IMU_CMD_PING);
    assert(parsed.payloadLength == 0U);
}

static void test_native_data_frame_is_lightweight(void)
{
    uint8_t bytes[EXTERNAL_IMU_PROTOCOL_NATIVE_MAX_FRAME_SIZE];
    ExternalIMUProtocol_NativeDataPayload payload = {
        .timeMs                  = 1234U,
        .angularRateMilliDps     = -250,
        .angleMilliDeg           = 90000,
        .normalizedAngleMilliDeg = 90000,
        .imuState                = 1U,
        .appState                = 3U,
        .calResult               = 2U,
        .flags                   = EXTERNAL_IMU_DATA_FLAG_ANGLE_VALID,
    };
    size_t length;

    length = ExternalIMUProtocol_EncodeNativeIMUData(
        &payload, bytes, sizeof(bytes));

    assert(EXTERNAL_IMU_PROTOCOL_NATIVE_DATA_PAYLOAD_SIZE == 20U);
    assert(length == 27U);
    assert(bytes[0] == 0xA5U);
    assert(bytes[1] == 0x5AU);
    assert(bytes[2] == 22U);
    assert(bytes[3] == EXTERNAL_IMU_FRAME_DATA);
    assert(bytes[4] == EXTERNAL_IMU_DATA_IMU_STATUS);
    assert(bytes[21] == payload.imuState);
    assert(bytes[22] == payload.appState);
    assert(bytes[23] == payload.calResult);
    assert(bytes[24] == EXTERNAL_IMU_DATA_FLAG_ANGLE_VALID);
}

static void test_angle_normalization_is_signed_180(void)
{
    assert(ExternalIMUProtocol_NormalizeAngleMilliDeg180(0) == 0);
    assert(ExternalIMUProtocol_NormalizeAngleMilliDeg180(179999) == 179999);
    assert(ExternalIMUProtocol_NormalizeAngleMilliDeg180(180000) == -180000);
    assert(ExternalIMUProtocol_NormalizeAngleMilliDeg180(270000) == -90000);
    assert(ExternalIMUProtocol_NormalizeAngleMilliDeg180(-181000) == 179000);
    assert(EXTERNAL_IMU_DATA_FLAG_ANGLE_VALID == 0x04U);
}

static void test_jy901_angle_frame(void)
{
    uint8_t frame[EXTERNAL_IMU_PROTOCOL_JY901_FRAME_SIZE];
    uint8_t sum = 0U;
    int16_t yawRaw;

    assert(ExternalIMUProtocol_EncodeJY901AngleFrame(
               90000, frame, sizeof(frame)) ==
           EXTERNAL_IMU_PROTOCOL_JY901_FRAME_SIZE);

    yawRaw = (int16_t)((uint16_t)frame[6] | ((uint16_t)frame[7] << 8));
    assert(frame[0] == 0x55U);
    assert(frame[1] == 0x53U);
    assert(yawRaw == 16384);

    for (size_t index = 0U; index < 10U; index++) {
        sum = (uint8_t)(sum + frame[index]);
    }
    assert(frame[10] == sum);
}

static void test_jy901_angle_frame_normalizes_large_positive_to_negative(void)
{
    uint8_t frame[EXTERNAL_IMU_PROTOCOL_JY901_FRAME_SIZE];
    int16_t yawRaw;

    assert(ExternalIMUProtocol_EncodeJY901AngleFrame(
               270000, frame, sizeof(frame)) ==
           EXTERNAL_IMU_PROTOCOL_JY901_FRAME_SIZE);

    yawRaw = (int16_t)((uint16_t)frame[6] | ((uint16_t)frame[7] << 8));
    assert(yawRaw == -16384);
}

int main(void)
{
    test_crc16_ccitt_reference();
    test_native_command_round_trip();
    test_native_data_frame_is_lightweight();
    test_angle_normalization_is_signed_180();
    test_jy901_angle_frame();
    test_jy901_angle_frame_normalizes_large_positive_to_negative();

    return 0;
}
