#ifndef WIFI_UPLOAD_OLD_H
#define WIFI_UPLOAD_OLD_H

#include <stdint.h>
#include "sensor_control.h"

#define WIFI_UPLOAD_DEVICE_ID_SIZE       8U
#define WIFI_UPLOAD_FREE_MAX_RECORDS     200U
#define WIFI_UPLOAD_DAMPED_MAX_RECORDS   150U
/* 保留原 WiFi 上传入口的 81 条上限；当前受迫表格采集上限为 80 条。 */
#define WIFI_UPLOAD_FORCED_MAX_RECORDS   81U

/*
 * WiFi 表格上传通信协议（兼容现有上位机）
 *
 * 以下为 STM32 经 USART2 交给 ESP32 的完整表格数据帧，单位均为字节。
 * USART2 使用 115200、8N1；末字节 0x13 为 ESP32 的 WiFi 转发标志。
 * 表格帧不额外添加 EE BE 帧头、长度字段或校验和。
 * device_id 为原样发送的 8 字节；命令码、计数、档位均为 uint8_t。
 * 所有浮点字段均为 IEEE 754 binary32（4 字节），按小端序发送，
 * 例如 1.0f 编码为 00 00 80 3F。记录序号 index 不上传。
 *
 * 1. 自由振荡，命令码 0x59
 *    [device_id:8][59:1][N:1][记录:8*N][13:1]
 *    偏移 0..7: 设备 ID；8: 命令码；9: 实际记录数 N。
 *    从偏移 10 开始，每条记录为 [amplitude:4][period:4]。
 *    按输入顺序发送；总长度 = 11 + 8*N，最大 1611 字节。
 *
 * 2. 阻尼振荡，命令码 0x5A
 *    [device_id:8][5A:1][N1:1][N2:1][N3:1]
 *    [档位1记录:8*N1][档位2记录:8*N2][档位3记录:8*N3][13:1]
 *    偏移 9/10/11 分别为档位 1/2/3 的实际记录数。
 *    数据从偏移 12 开始，每条记录为 [amplitude:4][period:4]。
 *    按档位 1、2、3 分组，组内保持输入顺序；其他档位不上传。
 *    总长度 = 13 + 8*(N1+N2+N3)，最大 1213 字节。
 *
 * 3. 受迫振荡，命令码 0x51
 *    [device_id:8][51:1][N:1][damping:1][记录:24*N][13:1]
 *    偏移 9: 筛选后的记录数 N；10: 所选阻尼档位（1、2 或 3）。
 *    数据从偏移 11 开始，每条记录依次为：
 *    [period*10:4][period:4][drive_period:4][amplitude:4]
 *    [phase_diff1:4][phase_diff2:4]
 *    首字段 period*10 为旧上位机兼容字段，必须保留。
 *    只上传指定档位，保持输入顺序；总长度 = 12 + 24*N，最大 1956 字节。
 *
 * 空表或筛选结果为空时仍发送头部和 0x13，所有计数为 0。
 * 接收端应根据命令码和计数字段计算帧长；浮点数据中也可能出现 0x13，
 * 不能只按 0x13 分割表格帧。
 *
 * 发送流程：组包 -> HAL_Delay(100)
 *           -> ESP32_ack_data()（阻塞串口发送）。
 * 连接状态查询是另一条控制帧：EE BE E9 01 01 97 10，
 * 其中 97 为前五字节累加和的低 8 位，10 为 STM32 链路码。
 * 沿用原逻辑，查询后不以 wifi_connect_state 作为发送条件。
 * ESP32 上传成功通知为 6F 6B 0D 0A 13（"ok\r\n" + WiFi 标志），
 * 仍由 esp32.c 处理；本模块不等待成功通知，也不重试。
 */

typedef enum {
    WIFI_UPLOAD_SUBMITTED = 0,  /* 已调用发送接口，不代表上位机确认收到 */
    WIFI_UPLOAD_INVALID_PARAM  /* 参数无效，未查询连接状态、未发送 */
} WifiUploadResult_t;

/*
 * 公共参数约定：
 * - id 指向至少 8 字节的设备 ID。
 * - records 指向至少 count 条 Record_t；只读取 records[0..count-1]。
 * - count 为实际记录数（index_modeX - 1），不是下次写入索引。
 * - count == 0 时 records 可以为 NULL；各模式上限见上述宏定义。
 * - 调用期间输入数据必须保持有效且不被修改；模块不修改输入数据。
 * - 仅在主循环/任务上下文调用，包含延时和阻塞发送，不可在中断中调用。
 */
WifiUploadResult_t WifiUpload_Free(const uint8_t *id,
                                  const Record_t *records, uint16_t count);

/* count 为筛选前总条数，模块自动按阻尼档位 1/2/3 分组。 */
WifiUploadResult_t WifiUpload_Damped(const uint8_t *id,
                                    const Record_t *records, uint16_t count);

/* count 为筛选前总条数，damping 只能为 1、2 或 3。 */
WifiUploadResult_t WifiUpload_Forced(const uint8_t *id,
                                    const Record_t *records, uint16_t count,
                                    uint8_t damping);

#endif /* WIFI_UPLOAD_OLD_H */
