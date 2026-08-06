/*
 * ESP32.h
 *
 *  Created on: May 23, 2025
 *      Author: dhyf02
 */

#ifndef INC_ESP32_H_
#define INC_ESP32_H_


#include "main.h"

#define Y_CMD_HEAD 0XEE                                                  //帧头
#define Y_CMD_TAIL 0XFFEFFC                                            //帧尾
#define Y_QUEUE_MAX_SIZE 256   /*!< 指令接收缓冲区大小，根据需要调整，尽量设置大一些*/
#define Y_CMD_MAX_SIZE  256

typedef unsigned char y_qdata;
typedef unsigned short y_qsize;


typedef enum
{
	esp32_stm32=0x10,
	esp32_rs232,
	esp32_ble,
	esp32_wifi,
}link_code_t;

typedef enum
{
	wifi_state=0xEF,
	wifi_protocol=0xEE,
	wifi_sid_set=0xED,
	wifi_password_set=0xEC,
	wifi_service_IP_set=0xEB,
	wifi_service_port_set=0xEA,
	wifi_connect_state_set=0xE0,// wifi连接状态

	ble_state=0xDF,
	ble_name_set=0xDD,
	ble_data_write=0xDC,
	ble_mac_set=0xDB,
	ble_paring_set=0xDA,
	ble_connect_state=0xD0,//0:蓝牙关闭，1:未连接，2:已连接

	rs232_bond_set=0xCD,
	re232_check_bit=0xCC,

	get_config_info = 0xBE,// 获取配置信息
}cmd_code_t;


extern uint8_t ble_state_flag;
extern uint8_t wifi_state_flag;


void stm32_to_esp32_data(cmd_code_t cmd_code,uint8_t *p,uint16_t len,link_code_t link_code);
void ESP32_ack_data(uint8_t *p ,uint16_t len);
void ESP32_232_communication_task_callback(void);
void y_queue_push(y_qdata y_data);
void y_queue_reset(void);
uint8_t sum_low_8bits(uint8_t *numbers, uint16_t count);

#endif /* INC_ESP32_H_ */
