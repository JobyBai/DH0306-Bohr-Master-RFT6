#ifndef BLE_TABLE_UPLOAD_H
#define BLE_TABLE_UPLOAD_H

#include "main.h"   /* HAL, huart2 */
#include <stdint.h>

/* ------------------------------------------------------------------ *
 * 可配置项                                                             *
 * ------------------------------------------------------------------ */
#define BLE_UPLOAD_MAX_RETRY   5      /* 单行最大重试次数              */
#define BLE_UPLOAD_TX_BUF_SIZE 320    /* 帧头(5B)+载荷+校验(1B)+尾(3B) */

/* ------------------------------------------------------------------ *
 * 上传状态（只读，外部可查询）                                          *
 * ------------------------------------------------------------------ */
typedef enum {
    BLE_UPLOAD_IDLE = 0,   /* 空闲 / 未启动   */
    BLE_UPLOAD_SENDING,    /* 正在发送        */
    BLE_UPLOAD_DONE,       /* 全部发送完成    */
    BLE_UPLOAD_ERROR,      /* 超过重试次数    */
} BleUploadState_t;

/* ------------------------------------------------------------------ *
 * 振荡数据类型选择                                                      *
 * ------------------------------------------------------------------ */
typedef enum {
    BLE_MODE_FREE   = 0,   /* 自由振荡数据   free_record_buf   */
    BLE_MODE_DAMPED = 1,   /* 阻尼振荡数据   damped_record_buf */
    BLE_MODE_FORCED = 2,   /* 受迫振荡数据   forced_record_buf */
} BleUploadMode_t;


/* ------------------------------------------------------------------ *
 * 公开接口                                                             *
 * ------------------------------------------------------------------ */

/**
 * @brief  启动振荡数据上传
 * @param  mode  BLE_MODE_FREE / BLE_MODE_DAMPED / BLE_MODE_FORCED
 *               决定标题字符串和数据来源缓冲区
 *         若上传正在进行则忽略本次调用
 */
void BleUpload_Start(BleUploadMode_t mode);

/**
 * @brief  在 ESP32_232_data_tx 收到应答时调用
 * @param  success  1=发送成功(0x0A)  0=发送失败(0x0B)
 */
void BleUpload_OnAck(uint8_t success);

/**
 * @brief  查询当前上传状态
 */
BleUploadState_t BleUpload_GetState(void);

#endif /* BLE_TABLE_UPLOAD_H */