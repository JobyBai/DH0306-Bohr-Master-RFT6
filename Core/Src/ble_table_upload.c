#include "ble_table_upload.h"
#include "usart.h"    /* huart2 */
#include <stdio.h>
#include <string.h>
#include "sensor_control.h"
#include "hmi_control.h"
#include "hmi_driver.h"


/* ------------------------------------------------------------------ *
 * 协议常量                                                             *
 * ------------------------------------------------------------------ */
#define SEP          0x2C   /* 表格分隔符 ','                          */
#define CR           0x0D
#define LF           0x0A
#define BLE_FLAG     0x12   /* 串口转发蓝牙标志位，帧尾追加             */

/* 外层协议头 */
#define FRAME_HDR0        0xEE
#define FRAME_HDR1        0xBE
#define FRAME_FUNC        0x11   /* 功能码：数据传输（所有帧统一使用）    */
#define FRAME_ENC         0x01   /* 编码方式：UTF-8                      */


/* 各模式对应的 UTF-8 标题（自由 / 阻尼 / 受迫） */
static const char * const s_title_utf8[3] = {
    /* 自由振荡数据 */
    "\xE8\x87\xAA\xE7\x94\xB1\xE6\x8C\xAF\xE8\x8D\xA1\xE6\x95\xB0\xE6\x8D\xAE",
    /* 阻尼振荡数据 */
    "\xE9\x98\xBB\xE5\xB0\xBC\xE6\x8C\xAF\xE8\x8D\xA1\xE6\x95\xB0\xE6\x8D\xAE",
    /* 受迫振荡数据 */
    "\xE5\x8F\x97\xE8\xBF\xAB\xE6\x8C\xAF\xE8\x8D\xA1\xE6\x95\xB0\xE6\x8D\xAE",
};

// /* 各模式对应的缓冲区指针 */
// static const Record_t * const s_record_buf[3] = {
//     free_record_buf,
//     damped_record_buf,
//     forced_record_buf,
// };
// /* 各模式对应的索引变量指针 */
// static const uint16_t * const s_index[3] = {
//     &index_mode1,
//     &index_mode2,
//     &index_mode3,
// };

Record_t ble_free_record_buf[200];// 用于蓝牙上传的自由振荡测量记录
uint8_t ble_free_len = 0;// 用于蓝牙上传的自由振荡测量记录索引
Record_t ble_damped_record_buf[200];// 用于蓝牙上传的阻尼振荡测量记录
uint8_t ble_damped_len = 0;// 用于蓝牙上传的阻尼振荡测量记录索引
Record_t ble_forced_record_buf[200];// 用于蓝牙上传的迫迫振荡测量记录
uint8_t ble_forced_len = 0;// 用于蓝牙上传的迫迫振荡测量记录索引


/* ------------------------------------------------------------------ *
 * 模块内部状态                                                          *
 * ------------------------------------------------------------------ */
static struct {
    BleUploadState_t state;
    BleUploadMode_t  mode;          /* 模式选择 */
    int              current_row;
    int              retry_count;
    int              total_rows;
    uint8_t          buf[BLE_UPLOAD_TX_BUF_SIZE];
    uint16_t         buf_len;
} s_tx;



/* ------------------------------------------------------------------ *
 * 内部工具：追加单字节                                                  *
 * ------------------------------------------------------------------ */
static inline uint16_t append_byte(uint8_t *buf, uint16_t pos, uint8_t b)
{
    buf[pos++] = b;
    return pos;
}

/* ------------------------------------------------------------------ *
 * 内部工具：追加字节串                                                  *
 * ------------------------------------------------------------------ */
static inline uint16_t append_str(uint8_t *buf, uint16_t pos,
                                  const char *s)
{
    uint16_t len = (uint16_t)strlen(s);
    memcpy(buf + pos, s, len);
    return pos + len;
}

/* ------------------------------------------------------------------ *
 * 内部：校验和（功能码 ~ 数据末尾，低8位累加和）                         *
 * ------------------------------------------------------------------ */
static uint8_t sum_low_8bits(uint8_t *numbers, uint16_t count)
{
    uint32_t total = 0;
    for (uint16_t i = 0; i < count; i++) {
        total += numbers[i];
    }
    return (uint8_t)(total & 0xFF);
}

/* ------------------------------------------------------------------ *
 * 内部：构建数据载荷写入 payload，返回载荷长度                           *
 *                                                                     *
 * row_idx ==  0  : 第1数据行，内容为"自由振荡数据"标题（单列）           *
 * row_idx ==  1  : 第2数据行，内容为7列表头                             *
 * row_idx >= 2   : 数据行，对应 free_record_buf[row_idx-2]             *
 * ------------------------------------------------------------------ */
static uint16_t build_payload(uint8_t *payload, int row_idx)
{
    uint16_t pos = 0;
    char     tmp[32];

    /* 行号（从 1 开始）*/
    snprintf(tmp, sizeof(tmp), "%d", row_idx + 1);
    pos = append_str(payload, pos, tmp);

    if (row_idx == 0)
    {
        /* ── 第1行：标题"自由振荡数据"，单列 ─────────────────────── */
        pos = append_byte(payload, pos, SEP);
        pos = append_byte(payload, pos, '1');
        pos = append_byte(payload, pos, SEP);
        pos = append_str(payload, pos, s_title_utf8[s_tx.mode]);
    }
    else if(row_idx == 1)
    {
        if(s_tx.mode == 0)//自由振荡
        {
            pos = append_byte(payload, pos, SEP);
            pos = append_byte(payload, pos, '1');
            pos = append_byte(payload, pos, SEP);
        }
        else if(s_tx.mode == 1)//阻尼振荡
        {
            pos = append_byte(payload, pos, SEP);
            pos = append_byte(payload, pos, '1');
            pos = append_byte(payload, pos, SEP);
            pos = append_str(payload, pos, "\xE9\x98\xBB\xE5\xB0\xBC\xE6\xA1\xA3");//阻尼档
            pos = append_byte(payload, pos, SEP); pos = append_byte(payload, pos, '2'); pos = append_byte(payload, pos, SEP);
            snprintf(tmp, sizeof(tmp), "%d", damping_num);
            pos = append_str(payload, pos, tmp);
        }
        else if(s_tx.mode == 2)//受迫迫振荡
        {
            pos = append_byte(payload, pos, SEP);
            pos = append_byte(payload, pos, '1');
            pos = append_byte(payload, pos, SEP);
            pos = append_str(payload, pos, "\xE9\x98\xBB\xE5\xB0\xBC\xE6\xA1\xA3");//阻尼档
            pos = append_byte(payload, pos, SEP); pos = append_byte(payload, pos, '2'); pos = append_byte(payload, pos, SEP);
            snprintf(tmp, sizeof(tmp), "%d", filter_damping_num);
            pos = append_str(payload, pos, tmp);
        }
    }
    else if(row_idx == 2)
    {
       if(s_tx.mode == 0)//自由振荡
        {
            pos = append_byte(payload, pos, SEP);
            pos = append_byte(payload, pos, '1');
            pos = append_byte(payload, pos, SEP);
        }
        else if(s_tx.mode == 1)//阻尼振荡
        {
            pos = append_byte(payload, pos, SEP);
            pos = append_byte(payload, pos, '1');
            pos = append_byte(payload, pos, SEP);
            pos = append_str(payload, pos, "\xE9\x98\xBB\xE5\xB0\xBC\xE6\xA1\xA3\xE8\xB0\x83\xE8\x8A\x82\xE5\x80\xBC");//阻尼档调节值
            pos = append_byte(payload, pos, SEP); pos = append_byte(payload, pos, '2'); pos = append_byte(payload, pos, SEP);
            if(damping_num == 1){
                 snprintf(tmp, sizeof(tmp), "%d", damping1);
            }else if(damping_num == 2){
                 snprintf(tmp, sizeof(tmp), "%d", damping2);
            }else if(damping_num == 3){
                 snprintf(tmp, sizeof(tmp), "%d", damping3);
            }       
            pos = append_str(payload, pos, tmp);
        }
        else if(s_tx.mode == 2)//受迫迫振荡
        {
            pos = append_byte(payload, pos, SEP);
            pos = append_byte(payload, pos, '1');
            pos = append_byte(payload, pos, SEP);
            pos = append_str(payload, pos, "\xE9\x98\xBB\xE5\xB0\xBC\xE6\xA1\xA3\xE8\xB0\x83\xE8\x8A\x82\xE5\x80\xBC");//阻尼档调节值
            pos = append_byte(payload, pos, SEP); pos = append_byte(payload, pos, '2'); pos = append_byte(payload, pos, SEP);
            if(filter_damping_num == 1){
                 snprintf(tmp, sizeof(tmp), "%d", damping1);
            }else if(filter_damping_num == 2){
                 snprintf(tmp, sizeof(tmp), "%d", damping2);
            }else if(filter_damping_num == 3){
                 snprintf(tmp, sizeof(tmp), "%d", damping3);
            }       
            pos = append_str(payload, pos, tmp);
        }
    }
    else if (row_idx == 3)
    {
        if(s_tx.mode == 0 || s_tx.mode == 1)//自由振荡和阻尼振荡
        {
             /* 7列表头（UTF-8）──────────────────────────────── */
            static const char * const headers[3] = {
                "\xE5\xBA\x8F\xE5\x8F\xB7",                           /* 序号      */
                "\xE6\x8C\xAF\xE5\xB9\x85",                           /* 振幅      */
                "\xE5\x91\xA8\xE6\x9C\x9F",                           /* 周期      */
            };
            for (int col = 0; col < 3; col++)
            {
                pos = append_byte(payload, pos, SEP);
                snprintf(tmp, sizeof(tmp), "%d", col + 1);
                pos = append_str(payload, pos, tmp);
                pos = append_byte(payload, pos, SEP);
                pos = append_str(payload, pos, headers[col]);
            }
        }
        else if(s_tx.mode == 2)//受迫迫振荡
        {
             /* 7列表头（UTF-8）──────────────────────────────── */
            static const char * const headers[6] = {
                "\xE5\xBA\x8F\xE5\x8F\xB7",                           /* 序号      */
                "\xE6\x8C\xAF\xE5\xB9\x85",                           /* 振幅      */
                "\xE5\x91\xA8\xE6\x9C\x9F",                           /* 周期      */
                "\xE9\xA9\xB1\xE5\x8A\xA8\xE5\x91\xA8\xE6\x9C\x9F",  /* 驱动周期  */
                // "\xE9\x98\xBB\xE5\xB0\xBC\xE6\xA1\xA3\xE4\xBD\x8D",  /* 阻尼档位  */
                "\xE7\x9B\xB8\xE4\xBD\x8D\xE5\xB7\xAE\x31",          /* 相位差1   */
                "\xE7\x9B\xB8\xE4\xBD\x8D\xE5\xB7\xAE\x32",          /* 相位差2   */
            };
            for (int col = 0; col < 6; col++)
            {
                pos = append_byte(payload, pos, SEP);
                snprintf(tmp, sizeof(tmp), "%d", col + 1);
                pos = append_str(payload, pos, tmp);
                pos = append_byte(payload, pos, SEP);
                pos = append_str(payload, pos, headers[col]);
            }
        }
      
    }
    else
    {
        /* ── 第5行起：数据行，对应 free_record_buf[row_idx-4] ──────── */
        // const Record_t *r = &s_record_buf[s_tx.mode][row_idx - 4];
        Record_t *r;
        if(s_tx.mode == 0){
            r = &ble_free_record_buf[row_idx - 4];
        }else if(s_tx.mode == 1){
            r = &ble_damped_record_buf[row_idx - 4];
        }else if(s_tx.mode == 2){
            r = &ble_forced_record_buf[row_idx - 4];
        }



        /* 列1: index */
        pos = append_byte(payload, pos, SEP); pos = append_byte(payload, pos, '1'); pos = append_byte(payload, pos, SEP);
        snprintf(tmp, sizeof(tmp), "%lu", row_idx-3);// 从1开始
        pos = append_str(payload, pos, tmp);

        /* 列2: amplitude */
        pos = append_byte(payload, pos, SEP); pos = append_byte(payload, pos, '2'); pos = append_byte(payload, pos, SEP);
        snprintf(tmp, sizeof(tmp), "%.4f", (double)r->amplitude);
        pos = append_str(payload, pos, tmp);

        /* 列3: period */
        pos = append_byte(payload, pos, SEP); pos = append_byte(payload, pos, '3'); pos = append_byte(payload, pos, SEP);
        snprintf(tmp, sizeof(tmp), "%.4f", (double)r->period);
        pos = append_str(payload, pos, tmp);

        /* 列4: drive_period */
        pos = append_byte(payload, pos, SEP); pos = append_byte(payload, pos, '4'); pos = append_byte(payload, pos, SEP);
        snprintf(tmp, sizeof(tmp), "%.4f", (double)r->drive_period);
        pos = append_str(payload, pos, tmp);

        // /* 列5: damping */
        // pos = append_byte(payload, pos, SEP); pos = append_byte(payload, pos, '5'); pos = append_byte(payload, pos, SEP);
        // snprintf(tmp, sizeof(tmp), "%u", r->damping);
        // pos = append_str(payload, pos, tmp);

        /* 列5: phase_diff1 */
        pos = append_byte(payload, pos, SEP); pos = append_byte(payload, pos, '5'); pos = append_byte(payload, pos, SEP);
        snprintf(tmp, sizeof(tmp), "%.4f", (double)r->phase_diff1);
        pos = append_str(payload, pos, tmp);

        /* 列6: phase_diff2 */
        pos = append_byte(payload, pos, SEP); pos = append_byte(payload, pos, '6'); pos = append_byte(payload, pos, SEP);
        snprintf(tmp, sizeof(tmp), "%.4f", (double)r->phase_diff2);
        pos = append_str(payload, pos, tmp);
    }

    return pos;
}

/* ------------------------------------------------------------------ *
 * 内部：构建完整协议帧写入 buf，返回总长度                               *
 *                                                                     *
 * 所有帧统一格式（功能码 0x11）：                                        *
 *  EE BE | 0x11 | N+1 | 0x01 | 数据内容 0D 0A | 校验和 | 0x12         *
 *                              ↑__数据部分(含0D0A)__↑                  *
 *  校验范围：功能码 + 数据长度 + 编码方式 + 数据（含 0D 0A）             *
 *  BLE_FLAG(0x12) 追加在校验和之后，不计入校验                          *
 * ------------------------------------------------------------------ */
static uint16_t build_frame(uint8_t *buf, int row_idx)
{
    /* ── Step1: 构建数据载荷到 buf[5..]，再追加 0D 0A ────────────── */
    uint8_t  *payload     = buf + 5;
    uint16_t  payload_len = build_payload(payload, row_idx);
    payload[payload_len++] = CR;
    payload[payload_len++] = LF;

    /* ── Step2: 填写帧头 ──────────────────────────────────────────
     * 数据长度字段 = 编码方式(1B) + 数据N字节，即 payload_len + 1    */
    uint16_t pos = 0;
    buf[pos++] = FRAME_HDR0;                        /* EE              */
    buf[pos++] = FRAME_HDR1;                        /* BE              */
    buf[pos++] = FRAME_FUNC;                        /* 0x11            */
    buf[pos++] = (uint8_t)(payload_len + 1);        /* 数据长度 N+1    */
    buf[pos++] = FRAME_ENC;                         /* 0x01 UTF-8      */
    pos += payload_len;                             /* 跳过已写载荷    */

    /* ── Step3: 校验和（功能码 ~ 数据末尾含 0D 0A）──────────────── *
     * 校验区间：buf[2](功能码) 到 buf[pos-1](LF)                     */
    uint8_t checksum = sum_low_8bits(&buf[0], pos);
    buf[pos++] = checksum;

    /* ── Step4: BLE 串口转发标志，不计入校验 ─────────────────────── */
    buf[pos++] = BLE_FLAG;

    return pos;
}

/* ------------------------------------------------------------------ *
 * 内部：发送当前行（通过 huart2）                                       *
 * ------------------------------------------------------------------ */
static void send_current_row(void)
{
    s_tx.buf_len = build_frame(s_tx.buf, s_tx.current_row);
    HAL_UART_Transmit(&huart2, s_tx.buf, s_tx.buf_len, 1000);
    s_tx.state = BLE_UPLOAD_SENDING;
}

/* ================================================================== *
 * 公开接口实现                                                          *
 * ================================================================== */

void BleUpload_Start(BleUploadMode_t mode)
{
    if (s_tx.state == BLE_UPLOAD_SENDING) {
        printf("BleUpload: 上传正在进行中，忽略重复启动\r\n");
        return;
    }
    s_tx.mode        = mode;                        /* ← 新增 */
    s_tx.current_row = 0;
    s_tx.retry_count = 0;
    // s_tx.total_rows = 2 + (int)(*s_index[mode]-1);// 总行数
    s_tx.state       = BLE_UPLOAD_IDLE;

    if(mode == BLE_MODE_FREE){//自由振荡
        for(int i = 0; i < index_mode1-1; i++){
            ble_free_record_buf[ble_free_len++] = free_record_buf[i];
        }
        s_tx.total_rows = 4 + ble_free_len; // 总行数
    }else if(mode == BLE_MODE_DAMPED){//阻尼振荡
        // 筛选出对应阻尼档位的记录数
        for(int i = 0; i < index_mode2-1; i++){
           if(damped_record_buf[i].damping == damping_num){
               ble_damped_record_buf[ble_damped_len++] = damped_record_buf[i];
           }
        }
        s_tx.total_rows = 4 + ble_damped_len; // 总行数
    }else if(mode == BLE_MODE_FORCED){// 受迫振荡
         // 筛选出对应迫档位的记录数
        for(int i = 0; i < index_mode3-1; i++){
           if(forced_record_buf[i].damping == filter_damping_num){
               ble_forced_record_buf[ble_forced_len++] = forced_record_buf[i];
           }
        }
        s_tx.total_rows = 4 + ble_forced_len; // 总行数
    }

    printf("BleUpload: 开始上传 [%s]，共 %d 行\r\n",
           s_title_utf8[mode], s_tx.total_rows);

    send_current_row();
}

void BleUpload_OnAck(uint8_t success)
{
    if (ble_transfer_cancel == 0) return;// 取消传输, 忽略响应
    if (s_tx.state != BLE_UPLOAD_SENDING) return;

    if (success)
    {
        SetScreen(0x0009);// 展示正在蓝牙传输页面
        printf("BleUpload: 第 %d 行发送成功\r\n", s_tx.current_row + 1);
        s_tx.retry_count = 0;
        s_tx.current_row++;

        if (s_tx.current_row >= s_tx.total_rows)
        {
            ble_transfer_cancel = 0;// 上传完成, 重置取消标志
            SetScreen(current_screen);// 返回上传页面
            s_tx.state = BLE_UPLOAD_DONE;
            printf("BleUpload: 全部数据上传完成！\r\n");
        }
        else
        {
            send_current_row();
        }
    }
    else
    {
        s_tx.retry_count++;
        printf("BleUpload: 第 %d 行发送失败，第 %d 次重试\r\n",
               s_tx.current_row + 1, s_tx.retry_count);

        if (s_tx.retry_count >= BLE_UPLOAD_MAX_RETRY)
        {
            s_tx.state = BLE_UPLOAD_ERROR;
            printf("BleUpload: 错误！第 %d 行连续失败 %d 次，上传已中止！\r\n",
                   s_tx.current_row + 1, BLE_UPLOAD_MAX_RETRY);
            return;
        }

        /* 重发：buf 内容未变，直接重传 */
        HAL_UART_Transmit(&huart2, s_tx.buf, s_tx.buf_len, 1000);
    }
}

BleUploadState_t BleUpload_GetState(void)
{
    return s_tx.state;
}
