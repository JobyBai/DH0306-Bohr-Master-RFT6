#include "sensor_control.h"
#include "hmi_driver.h"
#include "spi.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "hmi_control.h"
#include "drv_can.h"
#include "esp32.h"

// CAN发送
uint8_t CAN_SLAVE_ANSWER[8] = { 0x00, 0x20 }; // 从机应答指令
uint8_t CAN_SLAVE_ANSWER_AMPLITUDE[8] = { 0x00, 0x21 }; // 获取振幅应答
uint8_t CAN_SLAVE_ANSWER_PERIOD[8] = { 0x00, 0x22 }; // 获取周期应答
uint8_t CAN_SLAVE_ANSWER_PHASE_DIFF_1[8] = { 0x00, 0x23 }; // 获取相位差1应答
uint8_t CAN_SLAVE_ANSWER_PHASE_DIFF_2[8] = { 0x00, 0x24 }; // 获取相位差2应答
uint8_t CAN_SLAVE_ANSWER_ADC_ENCODER[8] = { 0x00, 0x26 }; // 获取ADC值和编码器值应答
uint8_t CAN_SLAVE_ANSWER_CLOSE_ADC_ENCODER[8] = { 0x00, 0x27 }; // 关闭角度ADC和编码值获取
uint8_t CAN_SLAVE_ANSWER_VOLTAGE[8] = { 0x00, 0x28 }; // 获取电压值应答
uint8_t CAN_SLAVE_ANSWER_SET_VOLTAGE[8] = { 0x00, 0x29 }; // 设置电压值应答

///* ───── DMA 环形缓冲区 ───── */
//static uint8_t dma_buf[SENSOR_DMA_BUF_SIZE];
//static uint16_t read_pos = 0; // 上次读取到的位置（读指针）

/* ───── 对外数据 ───── */
float period = 0.0f;      // 周期
float amplitude = 0.0f;   // 振幅
float phase_diff1 = 0.0f; // 相位差1
float phase_diff2 = 0.0f; // 相位差2
uint16_t index_mode1 = 1;
uint16_t index_mode2 = 1;
uint16_t index_mode3 = 1;
// uint8_t sensor_frame_len = 10; // 单帧总字节数
uint8_t page_flag = 1; //当前页面标志位
uint8_t measure_flag = 0; // 测量标志位 // 0:停止测量，1：自由振荡测量，2：阻尼振荡测量，3：受迫振荡测量
Record_t free_record_buf[200];   // 自由振荡测量记录缓存
Record_t damped_record_buf[200]; // 阻尼振荡测量记录缓存
Record_t forced_record_buf[200]; // 受迫振荡测量记录缓存
Record_t filtered_record_buf[200]; // 受迫振荡 筛选结果缓存

uint8_t output_str[128]; // 输出字符串缓存

ADC_Encoder_t adc_encoder_buf[ADC_ENCODER_BUF_SIZE]; // ADC值与编码器值环形队列
uint16_t adc_encoder_index = 0;                      // 环形队列写指针


float sensor_voltage = 0.0f; // 磁角度传感器电压值


/* ───── 解析并记录一帧 ───── */
void Process_Frame() {

	// if (frame[1] == SENSOR_MODE1_ID) {
	// 自由振荡测量
	if (page_flag == 1) {
		// 实时展示
		SetTextFloat(0x01, 0x07, period, 4, 1);
		SetTextFloat(0x01, 0x06, amplitude, 3, 1);

		// 表格展示，并保存
		if (measure_flag == 1) {
			if (index_mode1 > 200) {
				// 停止采集 or 标记完成
				measure_flag = 0;
				SetButtonValue(1, 3, 0); //自动关闭测量按钮
				return;
			}

			snprintf((char*) output_str, sizeof(output_str), "%u;%.1f;%.4f",
					index_mode1, amplitude, period);
			Record_Add(1, 1, output_str);
			free_record_buf[index_mode1 - 1].index = index_mode1;
			free_record_buf[index_mode1 - 1].amplitude = amplitude;
			free_record_buf[index_mode1 - 1].period = period;
			free_record_buf[index_mode1 - 1].drive_period = 0.0f;
			free_record_buf[index_mode1 - 1].damping = 0; // 无阻尼振荡
			free_record_buf[index_mode1 - 1].phase_diff1 = 0.0f;
			free_record_buf[index_mode1 - 1].phase_diff2 = 0.0f;
            biaoge_bottom(1, 1, index_mode1); // 展示对应行数
			AI_Report_Record(&free_record_buf[index_mode1 - 1], AI_FUNC_FREE); // 上报AI网关（02 F0）
            index_mode1++;
		}
	} else if (page_flag == 2) {

		// 实时展示
		SetTextFloat(0x02, 0x09, period, 4, 1);
		SetTextFloat(0x02, 0x0A, amplitude, 3, 1);
		// 表格展示，并保存
		if (measure_flag == 2) {
			if (index_mode2 > 150) {
				// 停止采集 or 标记完成
				measure_flag = 0;
				SetButtonValue(2, 4, 0); //自动关闭测量按钮
				return;
			}
			// snprintf((char *)output_str, sizeof(output_str), "%u;%.1f;%.4f",
			//          index_mode2, amplitude, period);
			// Record_Add(2, 1, output_str);
			damped_record_buf[index_mode2 - 1].index = index_mode2;
			damped_record_buf[index_mode2 - 1].amplitude = amplitude;
			damped_record_buf[index_mode2 - 1].period = period;
			damped_record_buf[index_mode2 - 1].drive_period = 0.0f;
			damped_record_buf[index_mode2 - 1].damping = damping_num; // 阻尼档位
            damped_record_buf[index_mode2 - 1].phase_diff1 = 0.0f;
            damped_record_buf[index_mode2 - 1].phase_diff2 = 0.0f;
            // biaoge_bottom(2,1,index_mode2);// 展示对应行数
           	AI_Report_Record(&damped_record_buf[index_mode2 - 1], AI_FUNC_DAMPED); // 上报AI网关（02 F1）
            index_mode2++; // 序号已上报完成后再自增，保证与本条记录一致
		}
	} else if (page_flag == 3) { // 当电机不转动的时候，也会显示周期和幅值
		SetTextFloat(0x03, 0x0C, period, 4, 1);
		SetTextFloat(0x03, 0x0D, amplitude, 3, 1);
		SetTextFloat(0x03, 0x0E, phase_diff1, 1, 1);
		SetTextFloat(0x03, 0x0F, phase_diff2, 1, 1);
	}
}

/* ───── 公开函数 ───── */


void Control_Init(void) {
	DRV_RegisterCanRxCallback(Control_CanRxCallback); // 注册CAN接收处理回调函数
}

// CAN接收处理回调函数
void Control_CanRxCallback(uint32_t ID, uint8_t *buf, uint8_t len) {
	if (ID != CAN_ID_SLAVE)
		return;
	uint8_t function = buf[1];  // 功能码
	switch (function) {
	case 0x20: // 从机应答
		break;
	case 0x21: // 获取振幅
		memcpy(&amplitude, &buf[2], sizeof(float));
    	// 获取到振幅之后，发送周期请求
    	DRV_CAN_SendData(CAN_ID_MASTER, CAN_SLAVE_ANSWER_PERIOD, 8);
		break;
	case 0x22: // 获取周期
		memcpy(&period, &buf[2], sizeof(float));
    	// 获取到周期之后，发送相位差1请求
    	DRV_CAN_SendData(CAN_ID_MASTER, CAN_SLAVE_ANSWER_PHASE_DIFF_1, 8);
		break;
	case 0x23: // 获取相位差1
		memcpy(&phase_diff1, &buf[2], sizeof(float));
		// 获取到相位差1之后，发送相位差2请求
		DRV_CAN_SendData(CAN_ID_MASTER, CAN_SLAVE_ANSWER_PHASE_DIFF_2, 8);
		break;
	case 0x24: // 获取相位差2
		memcpy(&phase_diff2, &buf[2], sizeof(float));
		// 获取到相位差2之后，处理数据
		Process_Frame();
		break;
    case 0x25:// 获取adc值和编码器值
        // buf[2][3] 为ADC采样值，buf[4][5] 为编码器值，均为uint16_t
        memcpy(&adc_encoder_buf[adc_encoder_index].adc_value, &buf[2],
                sizeof(uint16_t));
        memcpy(&adc_encoder_buf[adc_encoder_index].encoder_value, &buf[4],
                sizeof(uint16_t));
        // 环形写入：写满后回到头部，覆盖最旧数据
        adc_encoder_index = (adc_encoder_index + 1) % ADC_ENCODER_BUF_SIZE;
        break;
	case 0x28: // 获取电压值
		memcpy(&sensor_voltage, &buf[2], sizeof(float));
		SetTextFloat(0x000E, 0x03, sensor_voltage, 3, 1);
		break;
	case 0x29: // 设置电压值
		SetScreen(0x000F);// 弹窗提示成功
		break;
	default:
		break;
	}
}



/* ───── AI网关数据上报 ───── */
/**
 * @brief  按统一协议向AI网关上报一条测量记录
 *         帧格式：帧头(EE BE) + 功能码(02 Fx) + 数据长度(1B N) + 数据(NB) + 校验和(1B)
 *         数据部分为ASCII字符串、逗号分割，三种模式共用前缀"序号,振幅,摆盘周期"：
 *           AI_FUNC_FREE  (02 F0)：序号,振幅,摆盘周期
 *           AI_FUNC_DAMPED(02 F1)：序号,振幅,摆盘周期,阻尼档位
 *           AI_FUNC_FORCED(02 F2)：序号,振幅,摆盘周期,阻尼档位,驱动周期,相位差1,相位差2
 *         校验范围：帧头 ~ 数据末尾（不含校验和本身）
 * @param  rec      已填充好的记录（各模式对应的 xxx_record_buf 元素）
 * @param  func_low 功能码低字节，取 AI_FUNC_FREE / AI_FUNC_DAMPED / AI_FUNC_FORCED
 */
void AI_Report_Record(const Record_t *rec, uint8_t func_low) {
    // DMA发送期间缓冲区必须保持有效，故声明为static（三个调用点均在main loop中顺序执行，无重入）
    static uint8_t ai_msg[AI_MSG_BUF_SIZE];
    int ai_data_len;

    if (rec == NULL)
        return;

    if (huart1.gState != HAL_UART_STATE_READY)
        return; // 上一次DMA尚未发完，丢弃本帧，避免覆写正在发送的缓冲区

    ai_msg[0] = AI_FRAME_HDR0; // 帧头1
    ai_msg[1] = AI_FRAME_HDR1; // 帧头2
    ai_msg[2] = AI_FUNC_HIGH;  // 功能码高字节
    ai_msg[3] = func_low;      // 功能码低字节

    // 数据部分：ASCII逗号分割，预留5字节头部 + 1字节校验
    switch (func_low) {
    case AI_FUNC_FREE: // 序号,振幅,摆盘周期
        ai_data_len = snprintf((char*) &ai_msg[5], sizeof(ai_msg) - 6,
                "%u,%.1f,%.4f", (unsigned int) rec->index, rec->amplitude,
                rec->period);
        break;
    case AI_FUNC_DAMPED: // 序号,振幅,摆盘周期,阻尼档位
        ai_data_len = snprintf((char*) &ai_msg[5], sizeof(ai_msg) - 6,
                "%u,%.1f,%.4f,%u", (unsigned int) rec->index, rec->amplitude,
                rec->period, (unsigned int) rec->damping);
        break;
    case AI_FUNC_FORCED: // 序号,振幅,摆盘周期,阻尼档位,驱动周期,相位差1,相位差2
        ai_data_len = snprintf((char*) &ai_msg[5], sizeof(ai_msg) - 6,
                "%u,%.1f,%.4f,%u,%.4f,%.1f,%.1f", (unsigned int) rec->index,
                rec->amplitude, rec->period, (unsigned int) rec->damping,
                rec->drive_period, rec->phase_diff1, rec->phase_diff2);
        break;
    default:
        return; // 未知功能码，不发送
    }

    if (ai_data_len < 0) {
        ai_data_len = 0;
    } else if (ai_data_len > (int) (sizeof(ai_msg) - 6)) {
        ai_data_len = (int) (sizeof(ai_msg) - 6); // 截断保护，防止N字段溢出
    }

    ai_msg[4] = (uint8_t) ai_data_len; // 数据长度 N
    ai_msg[5 + ai_data_len] = sum_low_8bits(ai_msg, 5 + ai_data_len); // 校验和：帧头~数据末尾低8位累加
    HAL_UART_Transmit_DMA(&huart1, ai_msg, 6 + ai_data_len); // 向AI网关发送数据
}