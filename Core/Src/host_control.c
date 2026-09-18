#include "host_control.h"
#include "usart.h"
#include "string.h"
#include "esp32.h"
#include "main.h"
#include "sensor_control.h"
#include "esp32.h"
#include "vofa.h"
#include "drv_can.h"


typedef struct H_QUEUE {
	h_qsize h_head;                                                       //队列头
	h_qsize h_tail;                                                       //队列尾
	h_qsize h_data[H_QUEUE_MAX_SIZE];                                  //队列数据缓存区
} HQUEUE;

static HQUEUE h_que = { 0, 0, { 0 } };                                    //指令队列
static uint32_t h_cmd_state = 0;                                      //队列帧尾检测状态
static h_qsize h_cmd_pos = 0;                                         //当前指令指针位置

h_qsize h_size = 0;
uint8_t h_cmd_buffer[H_CMD_MAX_SIZE];

/*!
 *  \brief  清空指令数据
 */
void h_queue_reset(void) {
	h_que.h_head = h_que.h_tail = 0;
	h_cmd_pos = h_cmd_state = 0;
}
/*!
 * \brief  添加指令数据
 * \detial 串口接收的数据，通过此函数放入指令队列
 *  \param  _data 指令数据
 */
void h_queue_push(h_qdata h_data) {
	h_qsize h_pos = (h_que.h_head + 1) % H_QUEUE_MAX_SIZE;
	if (h_pos != h_que.h_tail)                                            //非满状态
			{
		h_que.h_data[h_que.h_head] = h_data;
		h_que.h_head = h_pos;
	}
}

//从队列中取一个数据
static void h_queue_pop(h_qdata *h_data) {
	if (h_que.h_tail != h_que.h_head)                                     //非空状态
			{
		*h_data = h_que.h_data[h_que.h_tail];
		h_que.h_tail = (h_que.h_tail + 1) % H_QUEUE_MAX_SIZE;
	}
}

//获取队列中有效数据个数
static h_qsize h_queue_size() {
	return ((h_que.h_head + H_QUEUE_MAX_SIZE - h_que.h_tail) % H_QUEUE_MAX_SIZE);
}

/*!
 *  \brief  从指令队列中获取到一条完整指令指针
 *  \param  cmd 指令接收缓冲区
 *  \param  buf_len 指令接收缓冲区大小
 *  \return  指令长度，0表示没有获取到完整指令
 */
// h_qsize h_queue_find_cmd(h_qdata *buffer,h_qsize buf_len)
// {
//     h_qsize cmd_size = 0;
//     h_qdata _data = 0;
//     while(h_queue_size()>0)
//     {
//         //取一个数据
//         h_queue_pop(&_data);
//         if(h_cmd_pos == 0 && _data != H_CMD_HEAD)                               //指令第一个字节必须是帧头，否则丢弃
//         {
//             continue;
//         }
//         //    LED2_ON;
//         if(h_cmd_pos<buf_len)                                           //防止缓冲区溢出
//             buffer[h_cmd_pos++] = _data;
//         h_cmd_state = ((h_cmd_state<<8)|_data);                           //拼接最近4个字节，组成一个32位数
//         //最近4个字节与帧尾匹配，得到完整帧
//         if(h_cmd_state==H_CMD_TAIL)
//         {
//             //LED2_ON;
//             cmd_size = h_cmd_pos;                                       //指令字节长度
//             h_cmd_state = 0;                                            //重新检测帧尾
//             h_cmd_pos = 0;                                              //复位指针指向
//             return cmd_size;
//         }
//     }
//     return 0;                                                         //没有形成完整的一帧
// }
h_qsize h_queue_find_cmd(h_qdata *buffer, h_qsize buf_len) {
	h_qsize h_cmd_size = 0;
	h_qdata h_data = 0;

	static uint16_t dataAllLength, n;
	uint8_t sum_low;

	while (h_queue_size() > 0) {
		//取一个数据
		h_queue_pop(&h_data);
		if (h_data != H_CMD_HEAD_1 && h_cmd_pos == 0) {
			continue;
		}
		if (h_data != H_CMD_HEAD_2 && h_cmd_pos == 1) {
			h_cmd_pos = 0;
		}
		if (h_cmd_pos == 4) {
			n = h_data;
			dataAllLength = n + 6; //数据长度n+帧头2+功能码2+数字长度+校验码+(连接码)
		}

		//放置缓冲区溢出
		if (h_cmd_pos < buf_len) {
			buffer[h_cmd_pos++] = h_data;
		}
		if (h_cmd_pos == dataAllLength) {
			h_cmd_size = h_cmd_pos;
			h_cmd_pos = 0;
			sum_low = sum_low_8bits(buffer, n + 5);
			if (sum_low != buffer[n + 5]) {
				h_cmd_pos = 0;
				return 0;
			}
			return h_cmd_size;
		}
	}
	return 0;
}


//变量

uint16_t collect_frequency = 1000;// 采集频率
uint8_t collect_flag = 0; // 采集标志 0:关闭 1:开启


// 处理上位机发送指令
void host_data_deal(uint8_t *data, uint16_t len) {
    if(data[2] != 0x20) return;
	switch (data[3]) {
	case 0x07:  // 开启双数据传输
		//通知采集模块开始发送数据
		DRV_CAN_SendData(CAN_ID_MASTER, CAN_SLAVE_ANSWER_ADC_ENCODER, 8);
		// 设置采集频率
		uint16_t fre = data[4] << 8 | data[5];
        if (fre < 1 || fre > 1000) {
            fre = 1000;
        }
		collect_frequency = 1000 / fre; // 设置采集频率
		collect_flag = 1;
		break;
	case 0x08:  // 关闭双数据传输
		//通知采集模块停止发送数据
		DRV_CAN_SendData(CAN_ID_MASTER, CAN_SLAVE_ANSWER_CLOSE_ADC_ENCODER, 8);
		collect_flag = 0;
		collect_frequency = 0;
		break;
	default:
		break;
	}
}

void host_communication_task_callback(void) {
	h_size = h_queue_find_cmd(h_cmd_buffer, H_CMD_MAX_SIZE);
	if (h_size > 6) {
		host_data_deal(h_cmd_buffer, h_size);
		memset(h_cmd_buffer, 0, h_size);
	}
}

ADC_Encoder_t collect_buf[50]; // 采集数据缓存区
uint8_t collect_buf_index = 0;
// 主函数中调用, 根据采集频率处理数据
// 关键点：仅在 adc_encoder_index 发生变化（环形队列有新采样写入）时，
// 对本次新增的每一个写入位置沿用原来的取余抽取逻辑，
// 每 collect_frequency 个采样点从 adc_encoder_buf 中抽取一个数据
void collect_data_task_callback(void) {
	static uint16_t last_index = 0;  // 上次处理到的环形队列写指针
	static uint8_t  last_flag  = 0;  // 上次的采集开关状态，用于检测启停

	// 采集开关状态变化时同步基准索引，避免用旧值产生错误差值
	if (collect_flag != last_flag) {
		last_flag  = collect_flag;
		last_index = adc_encoder_index;
	}
	if (!collect_flag || collect_frequency == 0) return;

	if (adc_encoder_index == last_index) return;  // 无新数据，直接返回，避免按主频重复抽取
	if(adc_encoder_index % collect_frequency == 0){ // 每 collect_frequency 个采样点抽取一个数据
		collect_buf[collect_buf_index++] = adc_encoder_buf[adc_encoder_index];// 保存下来
		// 缓存区满时重置索引, 并一口气发送数据
		if(collect_buf_index >= 49){
			collect_buf_index = 0;
			// 数据发送格式： 帧头(0xEE 0xBE) + 功能码（0x20 0x07） 数据包长度0xC9(4*50+1) 数据量固定50组值（0x32） adc_value1 adc_value2 encoder_value1 encoder_value2 ... + 校验和（1B）
			static uint8_t test_msg[207] = {0xEE,0xBE,0x20,0x07,0xC9,0x32};
			for (int i = 0; i < 50; i++) {
				test_msg[i*4+6] = collect_buf[i].adc_value >> 8;
				test_msg[i*4+7] = collect_buf[i].adc_value & 0xFF;
				test_msg[i*4+8] = collect_buf[i].encoder_value >> 8;
				test_msg[i*4+9] = collect_buf[i].encoder_value & 0xFF;
			}
			test_msg[206] = sum_low_8bits(test_msg, 206);
			HAL_UART_Transmit_DMA(&huart1, test_msg, 207); // 发送数据
		}
			// VofaSend_Task(adc_encoder_buf[rd].adc_value, adc_encoder_buf[rd].encoder_value);// 上位机测试代码
	}
	last_index = adc_encoder_index;
}

