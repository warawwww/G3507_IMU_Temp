#include "imu_cal_storage.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define IMU_CAL_STORAGE_MAGIC          (0x494D5543UL)
#define IMU_CAL_STORAGE_VERSION        (1U)
#define IMU_CAL_STORAGE_RESERVED_WORDS (3U)

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t length;
    uint32_t sequence;
    uint32_t flags;
    float biasRaw24;
    float scaleCorrection;
    float biasTemperatureC;
    uint32_t sampleCount;
    uint32_t reserved[IMU_CAL_STORAGE_RESERVED_WORDS];
    uint32_t crc32;
} IMU_CalStorage_Record;

static uint32_t g_nextSequence = 1U;

static uint32_t IMU_CalStorage_Crc32(
    const void *data, uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc         = 0xFFFFFFFFUL;
    uint32_t i;
    uint32_t bit;

    for (i = 0U; i < length; i++) {
        crc ^= (uint32_t)bytes[i];
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 1U) != 0U) {
                crc = (crc >> 1U) ^ 0xEDB88320UL;
            } else {
                crc >>= 1U;
            }
        }
    }

    return ~crc;
}

static uint32_t IMU_CalStorage_RecordCrc(
    const IMU_CalStorage_Record *record)
{
    return IMU_CalStorage_Crc32(
        record, (uint32_t)offsetof(IMU_CalStorage_Record, crc32));
}

static bool IMU_CalStorage_IsRecordValid(
    const IMU_CalStorage_Record *record)
{
    if (record->magic != IMU_CAL_STORAGE_MAGIC) {
        return false;
    }
    if (record->version != IMU_CAL_STORAGE_VERSION) {
        return false;
    }
    if (record->length != sizeof(IMU_CalStorage_Record)) {
        return false;
    }

    return record->crc32 == IMU_CalStorage_RecordCrc(record);
}

bool IMU_CalStorage_Load(IMU_CalStorage_Data *data)
{
    const IMU_CalStorage_Record *record =
        (const IMU_CalStorage_Record *)IMU_CAL_STORAGE_FLASH_ADDRESS;

    if (data == NULL) {
        return false;
    }
    if (!IMU_CalStorage_IsRecordValid(record)) {
        return false;
    }

    data->biasRaw24        = record->biasRaw24;
    data->scaleCorrection  = record->scaleCorrection;
    data->biasTemperatureC = record->biasTemperatureC;
    data->sampleCount      = record->sampleCount;

    if (record->sequence == 0xFFFFFFFFUL) {
        g_nextSequence = 1U;
    } else {
        g_nextSequence = record->sequence + 1U;
    }

    return true;
}

bool IMU_CalStorage_Save(const IMU_CalStorage_Data *data)
{
    IMU_CalStorage_Record record = {0};
    DL_FLASHCTL_COMMAND_STATUS status;
    uint32_t primask;

    if (data == NULL) {
        return false;
    }

    record.magic            = IMU_CAL_STORAGE_MAGIC;
    record.version          = IMU_CAL_STORAGE_VERSION;
    record.length           = sizeof(record);
    record.sequence         = g_nextSequence;
    record.biasRaw24        = data->biasRaw24;
    record.scaleCorrection  = data->scaleCorrection;
    record.biasTemperatureC = data->biasTemperatureC;
    record.sampleCount      = data->sampleCount;
    record.crc32            = IMU_CalStorage_RecordCrc(&record);

    primask = __get_PRIMASK();
    __disable_irq();

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL, IMU_CAL_STORAGE_FLASH_ADDRESS,
                                DL_FLASHCTL_REGION_SELECT_MAIN);
    status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL,
                                            IMU_CAL_STORAGE_FLASH_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    if (status == DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        status = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
            FLASHCTL, IMU_CAL_STORAGE_FLASH_ADDRESS, (uint32_t *)&record,
            sizeof(record) / sizeof(uint32_t),
            DL_FLASHCTL_REGION_SELECT_MAIN);
    }
    DL_FlashCTL_protectSector(FLASHCTL, IMU_CAL_STORAGE_FLASH_ADDRESS,
                              DL_FLASHCTL_REGION_SELECT_MAIN);

    if (primask == 0U) {
        __enable_irq();
    }

    if (status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        return false;
    }

    g_nextSequence++;
    if (g_nextSequence == 0U) {
        g_nextSequence = 1U;
    }

    return IMU_CalStorage_IsRecordValid(
        (const IMU_CalStorage_Record *)IMU_CAL_STORAGE_FLASH_ADDRESS);
}
