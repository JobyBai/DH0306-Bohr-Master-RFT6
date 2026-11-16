#include "wifi_upload_old.h"
#include "esp32.h"
#include <string.h>

#define WIFI_UPLOAD_CMD_FREE       0x59U
#define WIFI_UPLOAD_CMD_DAMPED     0x5AU
#define WIFI_UPLOAD_CMD_FORCED     0x51U
#define WIFI_UPLOAD_FLAG           0x13U
#define WIFI_UPLOAD_FLOAT_SIZE     4U
#define WIFI_UPLOAD_PAIR_SIZE      (2U * WIFI_UPLOAD_FLOAT_SIZE)
#define WIFI_UPLOAD_FORCED_SIZE    (6U * WIFI_UPLOAD_FLOAT_SIZE)
#define WIFI_UPLOAD_FREE_HEADER    (WIFI_UPLOAD_DEVICE_ID_SIZE + 2U)
#define WIFI_UPLOAD_DAMPED_HEADER  (WIFI_UPLOAD_DEVICE_ID_SIZE + 4U)
#define WIFI_UPLOAD_FORCED_HEADER  (WIFI_UPLOAD_DEVICE_ID_SIZE + 3U)

_Static_assert(sizeof(float) == WIFI_UPLOAD_FLOAT_SIZE,
               "WiFi upload requires 32-bit float");

static uint8_t valid_records(const uint8_t *id, const Record_t *records,
                             uint16_t count, uint16_t max_count)
{
    return id != NULL && (records != NULL || count == 0U) && count <= max_count;
}

/* 逐字节写入小端浮点，避免未对齐访问和结构体填充字节影响协议。 */
static uint8_t *append_float_le(uint8_t *ptr, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    *ptr++ = (uint8_t)bits;
    *ptr++ = (uint8_t)(bits >> 8);
    *ptr++ = (uint8_t)(bits >> 16);
    *ptr++ = (uint8_t)(bits >> 24);
    return ptr;
}

static uint8_t *append_pair(uint8_t *ptr, const Record_t *record)
{
    ptr = append_float_le(ptr, record->amplitude);
    return append_float_le(ptr, record->period);
}

static WifiUploadResult_t send_frame(uint8_t *frame, uint16_t length)
{
    frame[length - 1U] = WIFI_UPLOAD_FLAG;
    // HAL_Delay(100);
    ESP32_ack_data(frame, length);
    return WIFI_UPLOAD_SUBMITTED;
}

WifiUploadResult_t WifiUpload_Free(const uint8_t *id,
                                  const Record_t *records, uint16_t count)
{
    if (!valid_records(id, records, count, WIFI_UPLOAD_FREE_MAX_RECORDS)) {
        return WIFI_UPLOAD_INVALID_PARAM;
    }

    /* 校验数量后再分配栈缓冲区，长度有界，且不额外占用常驻 RAM。 */
    uint8_t frame[WIFI_UPLOAD_FREE_HEADER + count * WIFI_UPLOAD_PAIR_SIZE + 1U];
    memcpy(frame, id, WIFI_UPLOAD_DEVICE_ID_SIZE);
    frame[8] = WIFI_UPLOAD_CMD_FREE;
    frame[9] = (uint8_t)count;

    uint8_t *ptr = frame + WIFI_UPLOAD_FREE_HEADER;
    for (uint16_t i = 0; i < count; i++) {
        ptr = append_pair(ptr, &records[i]);
    }

    return send_frame(frame, (uint16_t)sizeof(frame));
}

WifiUploadResult_t WifiUpload_Damped(const uint8_t *id,
                                    const Record_t *records, uint16_t count)
{
    if (!valid_records(id, records, count, WIFI_UPLOAD_DAMPED_MAX_RECORDS)) {
        return WIFI_UPLOAD_INVALID_PARAM;
    }

    uint8_t counts[3] = { 0, 0, 0 };
    for (uint16_t i = 0; i < count; i++) {
        uint8_t damping = records[i].damping;
        if (damping >= 1U && damping <= 3U) {
            counts[damping - 1U]++;
        }
    }

    uint16_t total = counts[0] + counts[1] + counts[2];
    uint8_t frame[WIFI_UPLOAD_DAMPED_HEADER + total * WIFI_UPLOAD_PAIR_SIZE + 1U];
    memcpy(frame, id, WIFI_UPLOAD_DEVICE_ID_SIZE);
    frame[8] = WIFI_UPLOAD_CMD_DAMPED;
    memcpy(frame + 9, counts, sizeof(counts));

    uint8_t *ptr = frame + WIFI_UPLOAD_DAMPED_HEADER;
    for (uint8_t damping = 1; damping <= 3; damping++) {
        for (uint16_t i = 0; i < count; i++) {
            if (records[i].damping == damping) {
                ptr = append_pair(ptr, &records[i]);
            }
        }
    }

    return send_frame(frame, (uint16_t)sizeof(frame));
}

WifiUploadResult_t WifiUpload_Forced(const uint8_t *id,
                                    const Record_t *records, uint16_t count,
                                    uint8_t damping)
{
    if (!valid_records(id, records, count, WIFI_UPLOAD_FORCED_MAX_RECORDS)
        || damping < 1U || damping > 3U) {
        return WIFI_UPLOAD_INVALID_PARAM;
    }

    uint8_t match_count = 0;
    for (uint16_t i = 0; i < count; i++) {
        if (records[i].damping == damping) {
            match_count++;
        }
    }

    uint8_t frame[WIFI_UPLOAD_FORCED_HEADER
                  + match_count * WIFI_UPLOAD_FORCED_SIZE + 1U];
    memcpy(frame, id, WIFI_UPLOAD_DEVICE_ID_SIZE);
    frame[8] = WIFI_UPLOAD_CMD_FORCED;
    frame[9] = match_count;
    frame[10] = damping;

    uint8_t *ptr = frame + WIFI_UPLOAD_FORCED_HEADER;
    for (uint16_t i = 0; i < count; i++) {
        if (records[i].damping != damping) {
            continue;
        }

        /* 首字段是旧上位机使用的周期十倍值，不能替换为记录序号。 */
        ptr = append_float_le(ptr, records[i].period * 10.0f);
        ptr = append_float_le(ptr, records[i].period);
        ptr = append_float_le(ptr, records[i].drive_period);
        ptr = append_float_le(ptr, records[i].amplitude);
        ptr = append_float_le(ptr, records[i].phase_diff1);
        ptr = append_float_le(ptr, records[i].phase_diff2);
    }

    return send_frame(frame, (uint16_t)sizeof(frame));
}
