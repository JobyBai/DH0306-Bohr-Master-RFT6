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

// 引脚定义
#define A2_PORT GPIOC
#define A2_PIN GPIO_PIN_8

#define A1_PORT GPIOB
#define A1_PIN GPIO_PIN_14

// 宏封装
#define A2_HIGH()  HAL_GPIO_WritePin(A2_PORT,A2_PIN,GPIO_PIN_SET)
#define A2_LOW()   HAL_GPIO_WritePin(A2_PORT,A2_PIN,GPIO_PIN_RESET)

#define A1_HIGH()  HAL_GPIO_WritePin(A1_PORT,A1_PIN,GPIO_PIN_SET)
#define A1_LOW()   HAL_GPIO_WritePin(A1_PORT,A1_PIN,GPIO_PIN_RESET)

/* ───── 帧定义 ───── */
#define SENSOR_FRAME_HEADER     0xEE
#define SENSOR_MODE1_ID         0xBE
#define SENSOR_MODE2_ID         0xBB
#define SENSOR_FRAME_LEN        10      // 单帧总字节数

/* ───── DMA 环形缓冲区 ───── */
#define SENSOR_DMA_BUF_SIZE     256     // 必须是2的幂，方便取模

/* ───── 对外数据（按需暴露） ───── */
extern _Bool sensor_receive_flag;
extern uint8_t sensor_data[20];
extern uint16_t index_mode1;
extern uint16_t index_mode2;
extern uint16_t index_mode3;
extern uint8_t sensor_frame_len;// 单帧总字节数
extern uint8_t page_flag;       // 页面标志位
extern uint8_t measure_flag;    // 测量标志位// 0:停止测量，1：自由振荡测量，2：阻尼振荡测量，3：强迫振荡测量


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


/* ───── 接口函数 ───── */
void Sensor_Control_Init(void);
void Sensor_Control_Stop(void);
void Sensor_Control_Start_Mode1(void);

void Sensor_Control_Start_Mode2(void);
// void Sensor_Receive(void);
void Sensor_Poll(void);

#endif /* __SENSOR_CONTROL_H__ */
