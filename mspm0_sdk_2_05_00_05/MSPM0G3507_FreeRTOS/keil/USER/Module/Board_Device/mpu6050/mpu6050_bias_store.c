#include "mpu6050_bias_store.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/dl_flashctl.h>

#define MPU6050_BIAS_MAGIC       0x4D505542U
#define MPU6050_BIAS_VERSION     1U
#define MPU6050_BIAS_TAIL        0x42494153U
#define MPU6050_BIAS_LIMIT_DPS   50.0f

typedef struct {
    uint32_t magic;
    uint32_t version;
    float gyro_bias_dps[3];
    uint32_t reserved;
    uint32_t tail;
    uint32_t crc32;
} mpu6050_bias_record_t;

typedef char mpu6050_bias_record_word_aligned[
    ((sizeof(mpu6050_bias_record_t) % 8U) == 0U) ? 1 : -1];

static uint32_t calculate_crc32(const void *data, uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *) data;
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t byte_index;
    uint8_t bit_index;

    for (byte_index = 0U; byte_index < length; byte_index++) {
        crc ^= bytes[byte_index];
        for (bit_index = 0U; bit_index < 8U; bit_index++) {
            crc = (crc >> 1U) ^
                ((crc & 1U) != 0U ? 0xEDB88320U : 0U);
        }
    }

    return ~crc;
}

static bool bias_is_reasonable(const float bias[3])
{
    uint8_t axis;

    for (axis = 0U; axis < 3U; axis++) {
        if (!((bias[axis] > -MPU6050_BIAS_LIMIT_DPS) &&
                (bias[axis] < MPU6050_BIAS_LIMIT_DPS))) {
            return false;
        }
    }

    return true;
}

bool mpu6050_bias_load(float gyro_bias_dps[3])
{
    mpu6050_bias_record_t record;
    uint32_t expected_crc;

    if (gyro_bias_dps == NULL) {
        return false;
    }

    memcpy(&record, (const void *) MPU6050_BIAS_FLASH_ADDRESS,
        sizeof(record));

    expected_crc = calculate_crc32(&record,
        (uint32_t) offsetof(mpu6050_bias_record_t, crc32));
    if ((record.magic != MPU6050_BIAS_MAGIC) ||
        (record.version != MPU6050_BIAS_VERSION) ||
        (record.tail != MPU6050_BIAS_TAIL) ||
        (record.crc32 != expected_crc) ||
        !bias_is_reasonable(record.gyro_bias_dps)) {
        return false;
    }

    gyro_bias_dps[0] = record.gyro_bias_dps[0];
    gyro_bias_dps[1] = record.gyro_bias_dps[1];
    gyro_bias_dps[2] = record.gyro_bias_dps[2];

    return true;
}

bool mpu6050_bias_save(const float gyro_bias_dps[3])
{
    mpu6050_bias_record_t record __attribute__((aligned(8)));
    float verify_bias[3];
    DL_FLASHCTL_COMMAND_STATUS command_status;
    uint32_t saved_primask;

    if ((gyro_bias_dps == NULL) || !bias_is_reasonable(gyro_bias_dps)) {
        return false;
    }

    record.magic = MPU6050_BIAS_MAGIC;
    record.version = MPU6050_BIAS_VERSION;
    record.gyro_bias_dps[0] = gyro_bias_dps[0];
    record.gyro_bias_dps[1] = gyro_bias_dps[1];
    record.gyro_bias_dps[2] = gyro_bias_dps[2];
    record.reserved = 0U;
    record.tail = MPU6050_BIAS_TAIL;
    record.crc32 = calculate_crc32(&record,
        (uint32_t) offsetof(mpu6050_bias_record_t, crc32));

    saved_primask = __get_PRIMASK();
    __disable_irq();

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(FLASHCTL, MPU6050_BIAS_FLASH_ADDRESS,
        DL_FLASHCTL_REGION_SELECT_MAIN);
    command_status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL,
        MPU6050_BIAS_FLASH_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);

    if (command_status == DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(FLASHCTL, MPU6050_BIAS_FLASH_ADDRESS,
            DL_FLASHCTL_REGION_SELECT_MAIN);
        command_status =
            DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
                FLASHCTL, MPU6050_BIAS_FLASH_ADDRESS,
                (uint32_t *) &record,
                (uint32_t) (sizeof(record) / sizeof(uint32_t)),
                DL_FLASHCTL_REGION_SELECT_MAIN);
    }

    __set_PRIMASK(saved_primask);

    if (command_status != DL_FLASHCTL_COMMAND_STATUS_PASSED) {
        return false;
    }

    return mpu6050_bias_load(verify_bias) &&
        (verify_bias[0] == gyro_bias_dps[0]) &&
        (verify_bias[1] == gyro_bias_dps[1]) &&
        (verify_bias[2] == gyro_bias_dps[2]);
}
