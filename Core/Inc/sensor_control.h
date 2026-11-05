/**
 * @brief 传感器从机控制头文件
 * @author bai
 * @date 2026-04-14
 */

#ifndef __SENSOR_CONTROL_H__
#define __SENSOR_CONTROL_H__

#include "main.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>


#define     CAN_ID_MASTER            0x00A       // 主机id
#define     CAN_ID_SLAVE             0x00B       // 从机id


/* ───── 对外数据（按需暴露） ───── */
extern _Bool sensor_receive_flag;
extern uint8_t sensor_data[20];
extern uint16_t index_mode1;
extern uint16_t index_mode2;
extern uint16_t index_mode3;
// extern uint8_t sensor_frame_len;// 单帧总字节数
extern uint8_t page_flag;       // 页面标志位
extern uint8_t measure_flag;    // 测量标志位// 0:停止测量，1：自由振荡测量，2：阻尼振荡测量，3：强迫振荡测量
extern float sensor_voltage; // 磁角度传感器电压值


extern float period;      // 周期
extern float amplitude;   // 振幅
extern float phase_diff1; // 相位差1
extern float phase_diff2; // 相位差2

typedef struct
{
    uint32_t index;// 记录索引，从0开始递增
    float amplitude;// 振幅
    float period;// 周期
    float drive_period;// 驱动周期
    uint8_t damping;// 阻尼档位
    float phase_diff1;// 相位差1
    float phase_diff2;// 相位差2
} Record_t;

extern Record_t free_record_buf[200];// 自由振荡测量记录缓存
extern Record_t damped_record_buf[200];// 阻尼振荡测量记录缓存
extern Record_t forced_record_buf[200];// 强迫振荡测量记录缓存
extern Record_t filtered_record_buf[200]; // 受迫振荡 筛选结果缓存

extern uint8_t output_str[128]; // 输出字符串缓存

extern uint8_t CAN_SLAVE_ANSWER[8]; // 从机应答指令
extern uint8_t CAN_SLAVE_ANSWER_AMPLITUDE[8]; // 获取振幅应答
extern uint8_t CAN_SLAVE_ANSWER_PERIOD[8]; // 获取周期应答
extern uint8_t CAN_SLAVE_ANSWER_PHASE_DIFF_1[8]; // 获取相位差1应答
extern uint8_t CAN_SLAVE_ANSWER_PHASE_DIFF_2[8]; // 获取相位差2应答
extern uint8_t CAN_SLAVE_ANSWER_ADC_ENCODER[8]; // 获取ADC值和编码器值应答
extern uint8_t CAN_SLAVE_ANSWER_CLOSE_ADC_ENCODER[8]; // 关闭角度ADC和编码值获取
extern uint8_t CAN_SLAVE_ANSWER_VOLTAGE[8]; // 获取电压值应答
extern uint8_t CAN_SLAVE_ANSWER_SET_VOLTAGE[8]; // 设置电压值应答


#define ADC_ENCODER_BUF_SIZE 1000 // ADC与编码器数据环形队列长度
typedef struct
{
    uint16_t adc_value;     // ADC采样值
    uint16_t encoder_value; // 编码器值
} ADC_Encoder_t;

// ... existing code ...
extern ADC_Encoder_t adc_encoder_buf[ADC_ENCODER_BUF_SIZE]; // ADC值与编码器值环形队列
extern uint16_t adc_encoder_index;                          // 环形队列写指针

/* ───── AI网关上报协议 ───── */
// 帧格式：帧头(2B EE BE) + 功能码(2B 02 Fx) + 数据长度(1B N) + 数据(NB, ASCII逗号分割) + 校验和(1B)
#define AI_FRAME_HDR0     0xEE   // 帧头1
#define AI_FRAME_HDR1     0xBE   // 帧头2
#define AI_FUNC_HIGH      0x02   // 功能码高字节
#define AI_FUNC_FREE      0xF0   // 功能码低字节：自由振荡
#define AI_FUNC_DAMPED    0xF1   // 功能码低字节：阻尼振荡
#define AI_FUNC_FORCED    0xF2   // 功能码低字节：受迫振荡
#define AI_MSG_BUF_SIZE   64     // 单帧发送缓冲区大小（含帧头与校验和）

/* ───── 接口函数 ───── */
// ... existing code ...
void Control_Init(void);
void Control_CanRxCallback(uint32_t ID, uint8_t *buf, uint8_t len);
void Process_Frame(void);
void AI_Report_Record(const Record_t *rec, uint8_t func_low); // 向AI网关上报一条测量记录



#endif /* __SENSOR_CONTROL_H__ */
