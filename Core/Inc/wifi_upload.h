#ifndef __WIFI_UPLOAD_H__
#define __WIFI_UPLOAD_H__

#include "esp32.h"
#include "main.h"
#include <stdint.h>


/* ------------------------------------------------------------------ *
 * 可配置项                                                             *
 * ------------------------------------------------------------------ */
#define WIFI_UPLOAD_MAX_RETRY   5      /* 单行最大重试次数              */
#define WIFI_UPLOAD_TX_BUF_SIZE 256    /* 单次最大发送数据量 */

/* 外层协议头 */
#define FRAME_HDR0        0xEE
#define FRAME_HDR1        0xBE
#define FRAME_FUNC        0x10   /* 功能码：0x10    */
#define WIFI_FLAG         0x13   /* 连接码 wifi 0x13                      */
#define LF                0x0A  /* 数据层分隔符 */

/* ------------------------------------------------------------------ *
 * 上传状态（只读，外部可查询）                                          *
 * ------------------------------------------------------------------ */
typedef enum {
    WIFI_UPLOAD_IDLE = 0,   /* 空闲 / 未启动   */
    WIFI_UPLOAD_SENDING,    /* 正在发送        */
    WIFI_UPLOAD_DONE,       /* 全部发送完成    */
    WIFI_UPLOAD_ERROR,      /* 超过重试次数    */
} WifiUploadState_t;

typedef enum {
    WIFI_MODE_FREE   = 0,   /* 自由振荡数据   free_record_buf   */
    WIFI_MODE_DAMPED = 1,   /* 阻尼振荡数据   damped_record_buf */
    WIFI_MODE_FORCED = 2,   /* 受迫振荡数据   forced_record_buf */
} WifiUploadMode_t;



/* ------------------------------------------------------------------ *
 * 模块内部状态                                                          *
 * ------------------------------------------------------------------ */
static struct {
    WifiUploadState_t state;         /* 上传状态 */
    WifiUploadMode_t  mode;          /* 模式选择，用于区分不同类型的上传 */
    uint16_t           current_row;   /* 当前行索引 */
    uint8_t           retry_count;   /* 当前行重试次数 */
    uint16_t           total_rows;    /* 总行数 */
    uint8_t          buf[WIFI_UPLOAD_TX_BUF_SIZE]; /* 发送缓冲区 */
    uint16_t         buf_len;       /* 发送缓冲区当前长度 */
} s_tx;


void Wifi_Start_TX(void);
void Wifi_End_TX(void);
void WifiUpload_Start(WifiUploadMode_t mode);
void WifiUpload_Process(uint8_t *data, uint8_t len);

#endif