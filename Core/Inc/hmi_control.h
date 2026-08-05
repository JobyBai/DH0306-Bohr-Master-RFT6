/*
 * screen.h
 *
 *  Created on: Aug 22, 2025
 *      Author: dhyf02
 */

#ifndef INC_HMI_CONTROL_H_
#define INC_HMI_CONTROL_H_


#include "main.h"
#include "tim.h"
#include "usart.h"

extern uint8_t device_id[8];//设备ID
// wifi参数
extern uint8_t wlan_sid[60];
extern uint8_t wlan_password[60];
extern uint8_t wlan_service_IP[60];
extern uint8_t wlan_service_port[60];
extern uint8_t wifi_state_t[2];
extern uint8_t protocol_type[2]; //TCP-1    UDP-0;
extern uint8_t wifi_connect_state;// wifi连接状态 0未连接 1连接
extern uint8_t wifi_transfer_cancel;// 取消wifi传输, 1允许传输 2不允许传输

// 蓝牙参数
extern uint8_t ble_name[60];
extern uint8_t ble_mac[60];
extern uint8_t ble_paring[60];
extern uint8_t ble_transfer_cancel;// 取消蓝牙传输, 1允许传输 2不允许传输


// power指令集
extern uint8_t demagnetize_msg[6]; // 继电器位置：消磁
extern uint8_t demagnetize_open_msg[6]; // 继电器位置：消磁 开启消磁
extern uint8_t current_msg[6]; // 继电器位置：恒流源
extern uint8_t target_current_msg[6]; // 设置目标电流
extern uint8_t motor_cycle_msg[6]; // 设置电机周期
extern uint8_t motor_open_msg[6]; // 电机开启
extern uint8_t motor_close_msg[6]; // 电机关闭


// 阻尼1系数
extern uint16_t damping1;
// 阻尼2系数
extern uint16_t damping2;
// 阻尼3系数
extern uint16_t damping3;
// 阻尼档位
extern uint8_t damping_num;

extern uint8_t connect_type;// 连接方式 0x04: wifi 0x07: 蓝牙
extern uint16_t current_screen;// 当前屏幕
extern uint16_t jog_num;// 点动档位

// FilterByDampedNum 相关的展示状态，用于判断是否需要刷新表格
extern uint8_t  last_damping_num;  // 上一次展示的阻尼档位，0xFF表示"从未展示过"
extern uint8_t last_index_mode2;     // 上一次的数据总量
extern uint8_t filter_damping_num; // 受迫表格过滤的阻尼档位 


void touch_hmi_task_callback(void);
void GetFlashData(void);
void SendDampingCoefficient(uint8_t item);
void GetConfigInfo(void);
void FilterData(uint8_t target_damping);
void FilterByDampedNum(uint8_t num);


#endif /* INC_HMI_CONTROL_H_ */
