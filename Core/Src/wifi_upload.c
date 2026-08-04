#include "wifi_upload.h"
#include "sensor_control.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "hmi_control.h"
#include "esp32.h"
#include "hmi_driver.h"



Record_t wifi_free_record_buf[200];// 用于wifi上传的自由振荡测量记录
uint8_t wifi_free_len = 0;// 用于wifi上传的自由振荡测量记录索引
Record_t wifi_damped_record_buf[200];// 用于wifi上传的阻尼振荡测量记录
uint8_t wifi_damped_len = 0;// 用于wifi上传的阻尼振荡测量记录索引
Record_t wifi_forced_record_buf[200];// 用于wifi上传的迫迫振荡测量记录
uint8_t wifi_forced_len = 0;// 用于wifi上传的迫迫振荡测量记录索引


// wifi开始传输数据
void Wifi_Start_TX(void)
{
    uint8_t msg[]={0xEE,0xBE,0x10,0x06,0x01,0x73,0x74,0x61,0x72,0x74,0xF1,0x13};
    ESP32_ack_data(msg,12);
}

// wifi结束传输数据
void Wifi_End_TX(void)
{
    uint8_t msg[]={0xEE,0xBE,0x10,0x05,0x01,0x73,0x74,0x6F,0x70,0x88,0x13};
    ESP32_ack_data(msg,11);
}



/* ------------------------------------------------------------------ *
 * 内部：构建数据载荷写入 payload，返回载荷长度 ， 分别拼接不同类型的数据                          *
 * ------------------------------------------------------------------ */
static uint16_t build_payload(uint8_t *payload, uint16_t row_idx)
{
    uint16_t pos = 0;
    if(s_tx.mode == WIFI_MODE_FREE)// 构建自由振荡测量记录
    {
        // 写入功能码（1字节）
        payload[pos++] = 0x20;
        // 写入 device_id（8字节）
        memcpy(&payload[pos], device_id, sizeof(device_id));
        pos += sizeof(device_id);
        // 写入分隔符（1字节）
        payload[pos++] = LF;
        // 写入数据类型（1字节）
        payload[pos++] = 0x01;
        // 写入分隔符（1字节）
        payload[pos++] = LF;
        // 写入幅值（4字节）
        memcpy(&payload[pos], (uint8_t*) &wifi_free_record_buf[row_idx].amplitude,
								sizeof(float));
		pos += sizeof(float);
        // 写入周期（4字节）
        memcpy(&payload[pos], (uint8_t*) &wifi_free_record_buf[row_idx].period,
								sizeof(float));
		pos += sizeof(float);

    }
    // else if(s_tx.mode == WIFI_MODE_DAMPED)
    // {
    //     pos = build_damped_record(payload);
    // }
    // else if(s_tx.mode == WIFI_MODE_FORCED)
    // {
    //     pos = build_forced_record(payload);
    // }
    return pos;
}


/* ------------------------------------------------------------------ *
 * 内部：构建完整协议帧写入 buf，返回总长度                               *
 *                                                                     *
 * 所有帧统一格式（功能码 0x10）：                                        *
 *  EE BE | 0x10 | N+1 | 数据内容（包含子功能码） | 校验和 | 0x13        *
 *                                             *
 *  校验范围：帧头 + 功能码 + 数据长度 + 数据             *
 *  WIFI_FLAG(0x13) 追加在校验和之后，不计入校验                          *
 * ------------------------------------------------------------------ */
static uint16_t build_frame(uint8_t *buf, uint16_t row_idx)
{
    /* ── Step1: 构建数据载荷到 buf ────────────── */
    uint8_t  *payload     = buf;
    uint16_t  payload_len = build_payload(payload, row_idx);

    /* ── Step2: 填写帧头 ──────────────────────────────────────────
     * 数据长度字段 = 编码方式(1B) + 数据N字节，即 payload_len + 1    */
    uint16_t pos = 0;
    buf[pos++] = FRAME_HDR0;                        /* EE              */
    buf[pos++] = FRAME_HDR1;                        /* BE              */
    buf[pos++] = FRAME_FUNC;                        /* 0x10            */
    buf[pos++] = (uint8_t)(payload_len);        /* 数据长度 N+1    */
    pos += payload_len;                             /* 跳过已写载荷    */

    /* ── Step3: 校验和（功能码 ~ 数据末尾含 0D 0A）──────────────── *
     * 校验区间：buf[2](功能码) 到 buf[pos-1](LF)                     */
    uint8_t checksum = sum_low_8bits(&buf[0], pos);
    buf[pos++] = checksum;

    /* ── Step4: wifi传输标志 ─────────────────────── */
    buf[pos++] = WIFI_FLAG;

    return pos;
}

/* ------------------------------------------------------------------ *
 * 内部：发送当前行（通过 huart2）                                       *
 * ------------------------------------------------------------------ */
static void send_current_row(void)
{
    s_tx.buf_len = build_frame(s_tx.buf, s_tx.current_row);
    HAL_UART_Transmit(&huart2, s_tx.buf, s_tx.buf_len, 1000);
    s_tx.state = WIFI_UPLOAD_SENDING;
}

/**
 * @brief  启动wifi上传
 * @param  mode  WIFI_MODE_FREE / WIFI_MODE_DAMPED / WIFI_MODE_FORCED
 *               决定标题字符串和数据来源缓冲区
 *         若上传正在进行则忽略本次调用
 */
void WifiUpload_Start(WifiUploadMode_t mode)
{
    if (s_tx.state == WIFI_UPLOAD_SENDING) {
        printf("WifiUpload: 上传正在进行中，忽略重复启动\r\n");
        return;
    }
    s_tx.mode        = mode;                        /* ← 新增 */
    s_tx.current_row = 0;
    s_tx.retry_count = 0;
    // s_tx.total_rows = 2 + (int)(*s_index[mode]-1);// 总行数
    s_tx.state       = WIFI_UPLOAD_IDLE;

    if(mode == WIFI_MODE_FREE){//自由振荡
        for(int i = 0; i < index_mode1-1; i++){
            wifi_free_record_buf[wifi_free_len++] = free_record_buf[i];
        }
        s_tx.total_rows = 4 + wifi_free_len; // 总行数
    }
    // else if(mode == WIFI_MODE_DAMPED){//阻尼振荡
    //     // 筛选出对应阻尼档位的记录数
    //     for(int i = 0; i < index_mode2-1; i++){
    //        if(damped_record_buf[i].damping == damping_num){
    //            ble_damped_record_buf[ble_damped_len++] = damped_record_buf[i];
    //        }
    //     }
    //     s_tx.total_rows = 4 + wifi_damped_len; // 总行数
    // }else if(mode == WIFI_MODE_FORCED){// 受迫振荡
    //      // 筛选出对应迫档位的记录数
    //     for(int i = 0; i < index_mode3-1; i++){
    //        if(forced_record_buf[i].damping == filter_damping_num){
    //            ble_forced_record_buf[ble_forced_len++] = forced_record_buf[i];
    //        }
    //     }
    //     s_tx.total_rows = 4 + ble_forced_len; // 总行数
    // }

    printf("WifiUpload: 开始上传 [%d]，共 %d 行\r\n",
           mode, s_tx.total_rows);

    send_current_row();
}


void WifiUpload_OnAck(uint8_t success)
{
    if (ble_transfer_cancel == 0) return;// 取消传输, 忽略响应
    if (s_tx.state != WIFI_UPLOAD_SENDING) return;

    if (success)
    {
        SetScreen(0x0009);// 展示正在蓝牙传输页面
        printf("WifiUpload: 第 %d 行发送成功\r\n", s_tx.current_row + 1);
        s_tx.retry_count = 0;
        s_tx.current_row++;

        if (s_tx.current_row >= s_tx.total_rows)
        {
            ble_transfer_cancel = 0;// 上传完成, 重置取消标志
            SetScreen(current_screen);// 返回上传页面
            s_tx.state = WIFI_UPLOAD_DONE;
            printf("WifiUpload: 全部数据上传完成！\r\n");
        }
        else
        {
            send_current_row();
        }
    }
    else
    {
        s_tx.retry_count++;
        printf("WifiUpload: 第 %d 行发送失败，第 %d 次重试\r\n",
               s_tx.current_row + 1, s_tx.retry_count);

        // 上传失败, 重试次数超过最大重试次数
        if (s_tx.retry_count >= WIFI_UPLOAD_MAX_RETRY)
        {
            s_tx.state = WIFI_UPLOAD_ERROR;
            SetScreen(0x000D);// 弹窗提示上传失败
            printf("WifiUpload: 错误！第 %d 行连续失败 %d 次，上传已中止！\r\n",
                   s_tx.current_row + 1, WIFI_UPLOAD_MAX_RETRY);
            return;
        }

        /* 重发：buf 内容未变，直接重传 */
        HAL_UART_Transmit(&huart2, s_tx.buf, s_tx.buf_len, 1000);
    }
}


/**
 * @brief  wifi数据交互指令处理
 */
 void WifiUpload_Process(uint8_t *data, uint8_t len)
{
    uint8_t cmd = data[4];// 子功能码
    switch(cmd){
        case 0x01:// 启动上传
             WifiUpload_Start(WIFI_MODE_FREE);
            break;
        default:
            break;
    }
}
