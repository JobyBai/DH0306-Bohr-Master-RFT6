/*
 * screen.c
 *
 *  Created on: Aug 22, 2025
 *      Author: dhyf02
 */

#include "hmi_control.h"
#include "ble_table_upload.h"
#include "cmd_process.h"
#include "cmd_queue.h"
#include "esp32.h"
#include "gpio.h"
#include "hmi_driver.h"
#include "math.h"
#include "sensor_control.h"
#include "sfud.h"
#include "string.h"
#include "usart.h"
#include "wifi_upload.h"

uint8_t device_id[8] = { 0x42, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36 }; // 设备ID
// wifi 相关参数
uint8_t wlan_sid[60] = { 0 };          // wifi SSID
uint8_t wlan_password[60] = { 0 };     // wifi 密码
uint8_t wlan_service_IP[60] = { 0 };   // 服务端IP
uint8_t wlan_service_port[60] = { 0 }; // 服务端端口
uint8_t wifi_state_t[2] = { 0, 1 };    // wifi状态 0-未连接 1-已连接
uint8_t protocol_type[2] = { 0, 1 };   // TCP-1    UDP-0;
uint8_t wifi_connect_state = 0;      // wifi连接状态 0:禁用 ，1:打开未连接， 2：连接成功 3：连接中
uint8_t wifi_transfer_type = 0; // wifi传输类型, 0不允许传输 1自由振荡上传 2阻尼振荡上传 3受迫振荡上传

// 蓝牙 相关参数
uint8_t ble_state_t[2] = { 0, 1 }; // 蓝牙状态 0-未连接 1-已连接
uint8_t ble_name[60] = { 0 };      // 蓝牙名称
uint8_t ble_mac[60] = { 0 };       // 蓝牙MAC
uint8_t ble_paring[60] = { 0 };    // 蓝牙配对码
uint8_t ble_transfer_cancel = 0; // 取消蓝牙传输, 1允许传输 2不允许传输

// power命令集
uint8_t demagnetize_msg[6] = { 0xAA, 0x08, 0x00, 0x00, 0x00, 0x55 }; // 继电器位置：消磁
// uint8_t square_wave_open_msg[6] = { 0xAA, 0x0B, 0x00, 0x00, 0x00, 0x55 }; //
// 开启方波 uint8_t square_wave_close_msg[6] = { 0xAA, 0x0C, 0x00, 0x00, 0x00,
// 0x55 }; // 关闭方波 uint8_t square_wave_amplitude_msg[6] = { 0xAA, 0x0A,
// 0x01, 0x41, 0x00, 0x55 }; // 设置方波幅值,1.65 单位：V
uint8_t demagnetize_open_msg[6] = { 0xAA, 0x0E, 0x00, 0x00, 0x00, 0x55 }; // 继电器位置：消磁 开启消磁
uint8_t current_msg[6] = { 0xAA, 0x09, 0x00, 0x00, 0x00, 0x55 }; // 继电器位置：恒流源
uint8_t target_current_msg[6] = { 0xAA, 0x6E, 0x00, 0x00, 0x00, 0x55 }; // 设置目标电流
uint8_t motor_cycle_msg[6] = { 0xAA, 0x1D, 0x01, 0x00, 0xFA, 0x55 }; // 设置电机周期
uint8_t motor_open_msg[6] = { 0xAA, 0x14, 0x00, 0x00, 0x00, 0x55 };  // 电机开启
uint8_t motor_close_msg[6] = { 0xAA, 0x15, 0x00, 0x00, 0x00, 0x55 }; // 电机关闭
uint8_t jog_msg[6] = { 0xAA, 0x1A, 0x00, 0x00, 0x00, 0x55 };         // 点动

// 阻尼1系数
uint16_t damping1 = 1;
// 阻尼2系数
uint16_t damping2 = 2;
// 阻尼3系数
uint16_t damping3 = 3;
// 阻尼档位
uint8_t damping_num = 1;
//受迫表格档位选择
uint8_t filter_damping_num = 0;

// 连接方式 0x04: wifi 0x07: 蓝牙
uint8_t connect_type = 0x04;
// 当前页面
uint16_t current_screen = 0x0000;
// 点动档位
uint16_t jog_num = 1;

// FilterByDampedNum 相关的展示状态，用于判断是否需要刷新表格
uint8_t  last_damping_num  = 0xFF;  // 上一次展示的阻尼档位，0xFF表示"从未展示过"
uint8_t last_index_mode2  = 0;     // 上一次的数据总量
uint32_t last_local_index  = 0;   // 新增：当前档位下已展示的行数

// 步进系数
const float step_table[] = { 0.001f, 0.01f, 0.1f, 1.0f };
uint8_t step_index = 0;     // 默认 0.001
float motor_cycle = 1.250f; // 电机周期 s

int8_t line_flag = -1;      // 选中行数
uint8_t index_filtered = 0; // 筛选结果数量
uint8_t damping_index_filtered = 0; // 阻尼筛选结果数量

void ProcessMessage(PCTRL_MSG msg, uint16_t size) {
	uint8_t cmd_type = msg->cmd_type;                // 命令类型
	uint8_t ctrl_msg = msg->ctrl_msg;                // 消息控制类型
	uint8_t control_type = msg->control_type;        // 控件类型
	uint16_t screen_id = PTR2U16(&msg->screen_id);   // 屏幕ID
	uint16_t control_id = PTR2U16(&msg->control_id); // 控件ID
	uint32_t value = PTR2U32(msg->param);            // 参数值
	uint16_t value1 = PTR2U16(msg->param);

	switch (cmd_type) {
	case NOTIFY_TOUCH_PRESS:   // 触摸按下通知
	case NOTIFY_TOUCH_RELEASE: // 触摸释放通知
		// NotifyTouchXY(cmd_buffer[1],PTR2U16(cmd_buffer+2),PTR2U16(cmd_buffer+4));
		break;
	case NOTIFY_WRITE_FLASH_OK: // 写FLASH成功
		// NotifyWriteFlash(1);
		break;
	case NOTIFY_WRITE_FLASH_FAILD: // 写FLASH失败
		//  NotifyWriteFlash(0);
		break;
	case NOTIFY_READ_FLASH_OK: // 读取FLASH成功
		//  NotifyReadFlash(1,cmd_buffer+2,size-6); //去除帧头帧尾
		break;
	case NOTIFY_READ_FLASH_FAILD: // 读取FLASH失败
		// NotifyReadFlash(0,0,0);
		break;
	case NOTIFY_READ_RTC: // 读取RTC时间
		// NotifyReadRTC(cmd_buffer[2],cmd_buffer[3],cmd_buffer[4],cmd_buffer[5],cmd_buffer[6],cmd_buffer[7],cmd_buffer[8]);
		break;
	case NOTIFY_CONTROL: {
		if (ctrl_msg == MSG_GET_CURRENT_SCREEN) // 屏幕ID变化通知
		{
			NotifyScreen(screen_id); // 通知当前屏幕变化
		} else {
			switch (control_type) {
			case kCtrlknob:
				Notifyknob(screen_id, control_id, value1);
				break;
			case kCtrlButton: // 按钮控件
				NotifyButton(screen_id, control_id, msg->param[1]);
				break;
			case kCtrlText: // 文本控件
				NotifyText(screen_id, control_id, (const char*) msg->param);
				break;
			case kCtrlProgress: // 进度条控件
				NotifyProgress(screen_id, control_id, value);
				break;
			case kCtrlSlider: // 滑块控件
				NotifySlider(screen_id, control_id, value);
				break;
			case kCtrlMeter: // 仪表控件
				NotifyMeter(screen_id, control_id, value);
				break;
			case kCtrlMenu: // 菜单控件
				NotifyMenu(screen_id, control_id, msg->param[0], msg->param[1]);
				break;
			case kCtrlSelector: // 选择器控件
				NotifySelector(screen_id, control_id, msg->param[0]);
				break;
			case kCtrlDropList: // 下拉列表控件
				NotifySelector(screen_id, control_id, value);
				break;
			case kCtrlRTC: // 实时时间控件
				NotifyTimer(screen_id, control_id);
				break;
			case kCtrlRecord: // 记录按钮点击通知
				NotifyRecord(screen_id, control_id, msg->param[1]);

			default:
				break;
			}
		}
		break;
	}
	default:
		break;
	}
}

uint8_t current_screen_id = 0;

/*!
 *  \brief  当前屏幕变化通知
 *  \details  当前屏幕内容(通过GetScreen)或执行此函数时触发
 *  \param screen_id 当前屏幕ID
 */
void NotifyScreen(uint16_t screen_id) {
}

/*!
 *  \brief  触摸屏坐标响应
 *  \param press 1为按下，3为释放
 *  \param x x坐标
 *  \param y y坐标
 */
void NotifyTouchXY(uint8_t press, uint16_t x, uint16_t y) {
}

/*!
 *  \brief  旋钮控件通知
 *  \details  通过GetControlValue时执行此函数
 *  \param screen_id 屏幕ID
 *  \param control_id 控件ID
 *  \param value 值
 */

void Notifyknob(uint16_t screen_id, uint16_t control_id, uint16_t value) {
}

/*!
 *  \brief  更新界面
 */
void UpdateUI() {
	// wifi参数
	SetTextValue(4, 3, wlan_sid);
	SetTextValue(4, 4, wlan_password);
	SetTextValue(4, 5, wlan_service_IP);
	SetTextValue(4, 6, wlan_service_port);
	// 阻尼参数
	SetTextInt32(6, 3, damping1, 0, 1);
	SetTextInt32(6, 4, damping2, 0, 1);
	SetTextInt32(6, 5, damping3, 0, 1);
	// 受迫振荡相关
	SetTextFloat(3, 0x0B, motor_cycle, 3, 1);
	// 蓝牙参数
	SetTextValue(7, 3, ble_name);
	SetTextValue(7, 4, ble_mac);
	SetTextValue(7, 5, ble_paring);
}

/*!
 *  \brief  按钮控件通知
 *  \details  获取按钮状态变化(通过GetControlValue)时执行此函数
 *  \param screen_id 屏幕ID
 *  \param control_id 控件ID
 *  \param state 按钮状态，0释放1按下
 */
void NotifyButton(uint16_t screen_id, uint16_t control_id, uint8_t state) {
	if (screen_id == 0x0000) // 首页
			{
		switch (control_id) {
		case 0x01: // 自由震荡
			if (state == 0x01) {
				page_flag = 1;
				Sensor_Control_Start_Mode1();
				current_screen = 0x0001;
				// 切换继电器位置：消磁
				HAL_UART_Transmit(&huart4, demagnetize_msg, 6, 100);
			}
			break;
		case 0x02: // 阻尼振荡
			if (state == 0x01) {
				page_flag = 2;
				Sensor_Control_Start_Mode1();
				current_screen = 0x0002;
				// 切换继电器位置：恒流源
				HAL_UART_Transmit(&huart4, current_msg, 6, 100);
			}
			break;
		case 0x03: // 受迫振荡
			if (state == 0x01) {
				page_flag = 3;
				Sensor_Control_Start_Mode1();
				current_screen = 0x0003;
				// 切换继电器位置：恒流源
				HAL_UART_Transmit(&huart4, current_msg, 6, 100);
			}
			break;
		case 0x04: // 链接设置： 蓝牙 7 或 wifi 4
			if (state == 0x01) {
				SetScreen(connect_type);
			}
			break;
		default:
			break;
		}
	}
	if (screen_id == 0x0001) // 自由振荡
			{
		switch (control_id) {
		case 0x03: // 测量
			if (state == 0x01) {
				measure_flag = 1;
			} else {
				measure_flag = 0;
			}
			break;
		case 0x04: // 清空
			if (state == 0x01) {
				index_mode1 = 1;
				memset(free_record_buf, 0, sizeof(free_record_buf));
				Record_Clear(1, 1);
			}
			break;

		case 0x05: // 上传
			if (state == 0x01) {
				current_screen = 0x0001;
				if (connect_type == 0x04) // wifi上传
				{
					if(wifi_connect_state != 2) // 未连接wifi
					{
						SetScreen(0x0008);
						return;
					}
					wifi_transfer_type = 1;	
					Wifi_Start_TX();
				} else if (connect_type == 0x07) // 蓝牙上传
						{
					ble_transfer_cancel = 1;
					BleUpload_Start(BLE_MODE_FREE); // 自由振荡
				}
			}
			break;
		case 0x09: // 角度调整弹窗
			if (state == 0x01) {
				current_screen = 0x0001;
			}
			break;
		default:
			break;
		}
	}
	if (screen_id == 0x02) // 阻尼振荡
			{
		switch (control_id) {
		case 0x04: // 测量
			if (state == 0x01) {
				measure_flag = 2;
			} else {
				measure_flag = 0;
			}
			break;
		// case 0x03: // 清空所有
		// 	if (state == 0x01) {
		// 		index_mode2 = 1;
		// 		last_index_mode2 = 0;   // 强制下次 FilterByDampedNum 判定为"有新数据"，从而重新刷新
		// 		memset(damped_record_buf, 0, sizeof(damped_record_buf));
		// 		Record_Clear(2, 1);
		// 	}
		// 	break;
		case 0x03: // 清空当前选中档位的数据
			if (state == 0x01) {
				// 清空时需要先关闭测量，否则容易引起数据不一致
				measure_flag = 0;
				SetButtonValue(2, 4, 0);//自动关闭测量按钮
				uint32_t total = index_mode2 - 1; // 当前已保存的总条数
				uint32_t write_pos = 0;

				// 遍历全部数据，只保留 damping != damping_num 的项，原地压缩
				for (uint32_t read_pos = 0; read_pos < total; read_pos++) {
					if (damped_record_buf[read_pos].damping != damping_num) {
						if (write_pos != read_pos) {
							damped_record_buf[write_pos] = damped_record_buf[read_pos];
						}
						write_pos++;
					}
				}

				// 可选：把压缩后多余的尾部数据清零，避免残留脏数据造成误解
				memset(&damped_record_buf[write_pos], 0,
					sizeof(damped_record_buf[0]) * (total - write_pos));

				index_mode2 = write_pos + 1;   // 更新全局写入位置为压缩后的长度+1
				last_index_mode2 = 0;          // 强制下次判定为"有变化"，触发刷新
				last_local_index = 0;   // 新增
				last_damping_num = 0xFF;       // 防御性重置，确保一定会重新刷表

				// 主动刷新一次表格（此时当前档位应该已经没有数据了）
				FilterByDampedNum(damping_num);
			}
			break;

		case 0x05: // 上传
			if (state == 0x01) {
				if (index_mode2 > 151)
					return; // 防止越界, wifi传输最大数据量为200字节, 18字节为wifi标志,
							// 100字节为数据, 18字节为count
				current_screen = 0x0002;

				if (connect_type == 0x04) // wifi上传
					{
					if(wifi_connect_state != 2) // 未连接wifi
					{
						SetScreen(0x0008);
						return;
					}
					wifi_transfer_type = 2;	
					Wifi_Start_TX();
				} else if (connect_type == 0x07) // 蓝牙上传
						{
					ble_transfer_cancel = 1;
					BleUpload_Start(BLE_MODE_DAMPED); // 阻尼振荡
				}
			}
			break;
		case 0x06: // 跳转受迫振荡
		{
			if (state == 0x01) {
				page_flag = 3;
				Sensor_Control_Start_Mode1();
				// 切换继电器位置：恒流源
				HAL_UART_Transmit(&huart4, current_msg, 6, 100);
			}
		}
		case 0x0D: // 角度调整弹窗
			if (state == 0x01) {
				current_screen = 0x0002;
			}
			break;
		default:
			break;
		}
	}
	if (screen_id == 0x03) // 受迫振荡
			{
		switch (control_id) {
		case 0x01: // 开关
			if (state == 0x01) {
				Sensor_Control_Start_Mode2();
				HAL_UART_Transmit(&huart4, motor_open_msg, 6, 100);
			} else if (state == 0x00) {
				Sensor_Control_Start_Mode1();
				HAL_UART_Transmit(&huart4, motor_close_msg, 6, 100);
			}
			break;
		case 0x02: // 返回
			if (state == 0x01) {
				// 关闭电机
				SetButtonValue(3, 1, 0);
				HAL_UART_Transmit(&huart4, motor_close_msg, 6, 100);
			}
			break;
		case 0x03: // 速度+
			if (state == 0x01) {
				motor_cycle += step_table[step_index];
				if (motor_cycle > 2.0f)
					motor_cycle = 2.0f;                    // 加完再夹紧
				uint8_t int_part = (uint8_t) motor_cycle; // 整数秒
				uint16_t ms_part = (uint16_t) ((motor_cycle - int_part) * 1000
						+ 0.5f); // 小数转毫秒，四舍五入

				motor_cycle_msg[2] = int_part;
				motor_cycle_msg[3] = (uint8_t) (ms_part >> 8);   // 毫秒高字节
				motor_cycle_msg[4] = (uint8_t) (ms_part & 0xFF); // 毫秒低字节

				HAL_UART_Transmit(&huart4, motor_cycle_msg, 6, 100);
			}
			break;
		case 0x05: // 速度-
			if (state == 0x01) {
				motor_cycle -= step_table[step_index];
				if (motor_cycle < 1.0f)
					motor_cycle = 1.0f;                    // 减完再夹紧
				uint8_t int_part = (uint8_t) motor_cycle; // 整数秒
				uint16_t ms_part = (uint16_t) ((motor_cycle - int_part) * 1000
						+ 0.5f); // 小数转毫秒，四舍五入

				motor_cycle_msg[2] = int_part;
				motor_cycle_msg[3] = (uint8_t) (ms_part >> 8);   // 毫秒高字节
				motor_cycle_msg[4] = (uint8_t) (ms_part & 0xFF); // 毫秒低字节

				HAL_UART_Transmit(&huart4, motor_cycle_msg, 6, 100);
			}
			break;
		case 0x04: // 步进+
			if (state == 0x01) {
				step_index = (step_index + 1) % 3;
			}
			break;
		case 0x06: // 步进-
			if (state == 0x01) {
				step_index = (step_index + 2) % 3; // 等价于 -1 循环
			}
			break;
		case 0x07: // 保存
			if (state == 0x01) {
				if (index_mode3 > 80) {
					return;
				}
				snprintf((char*) output_str, sizeof(output_str),
						"%u;%.1f;%.4f;%.4f;%.1f;%.1f", index_mode3, amplitude,
						motor_cycle, period, phase_diff1, phase_diff2);
				Record_Add(5, 1, output_str);
				SetTextInt32(3, 0x0A, index_mode3, 0, 1);
				forced_record_buf[index_mode3 - 1].index = index_mode3;
				forced_record_buf[index_mode3 - 1].amplitude = amplitude;
				forced_record_buf[index_mode3 - 1].period = period;
				forced_record_buf[index_mode3 - 1].drive_period = motor_cycle;
				forced_record_buf[index_mode3 - 1].damping = damping_num;
				forced_record_buf[index_mode3 - 1].phase_diff1 = phase_diff1;
				forced_record_buf[index_mode3 - 1].phase_diff2 = phase_diff2;
				index_mode3++;
			}
			break;
		case 0x09: // 前往受迫表格页面
			if (state == 0x01) {
				SetTextInt32(5, 7, damping_num, 0, 1);
				FilterData(damping_num);
			}
			break;
		case 0x11: // 角度调整弹窗
			if (state == 0x01) {
				// 关闭电机
				SetButtonValue(3, 1, 0);
				HAL_UART_Transmit(&huart4, motor_close_msg, 6, 100);
				current_screen = 0x0003;
			}
			break;
		default:
			break;
		}
	}
	if (screen_id == 0x04) // wifi设置页
			{
		switch (control_id) {
		case 0x02: // wifi开关
			if (state == 0x01) {
				stm32_to_esp32_data(wifi_state, (uint8_t*) &wifi_state_t[1], 1,
						esp32_stm32);
			} else {
				stm32_to_esp32_data(wifi_state, (uint8_t*) &wifi_state_t[0], 1,
						esp32_stm32);
			}
			break;
		default:
			break;
		}
	}
	if (screen_id == 0x0005) // 受迫表格页面
			{
		switch (control_id) {
		case 0x02:
			if (state == 0x01) {
				uint8_t count = index_mode3 - 1; // 实际记录数
				if (count == 0 || line_flag > count || line_flag < 1) { // 注意：索引从1开始，所以 line_flag < 1
					line_flag = -1;
					return;
				}
				Record_Delete(5, 1, line_flag);

				for (uint8_t i = line_flag; i < count; i++) {
					forced_record_buf[i] = forced_record_buf[i + 1];
					forced_record_buf[i].index = i;
				}
				memset(&forced_record_buf[count], 0, sizeof(Record_t));
				line_flag = -1;
				index_mode3--; // 记录数-1，等价于count变为count-1

				Record_Clear(5, 1);
				uint8_t new_count = index_mode3 - 1;
				for (uint8_t i = 1; i <= new_count; i++) { // 从1开始遍历
					snprintf((char*) output_str, sizeof(output_str),
							"%lu;%.1f;%.4f;%.4f;%.1f;%.1f",
							forced_record_buf[i].index,
							forced_record_buf[i].amplitude,
							forced_record_buf[i].drive_period,
							forced_record_buf[i].period,
							forced_record_buf[i].phase_diff1,
							forced_record_buf[i].phase_diff2);
					Record_Add(5, 1, output_str);
				}

				SetTextInt32(3, 0x0A, index_mode3 - 1, 0, 1); // 显示实际记录数
			}
			break;
		case 0x03:
			if (state == 0x01) {
				line_flag = -1;
				index_mode3 = 1; // 重置回初始值1，而不是0
				index_filtered = 0;
				memset(forced_record_buf, 0, sizeof(forced_record_buf));
				memset(filtered_record_buf, 0, sizeof(filtered_record_buf));
				Record_Clear(5, 1);
				SetTextInt32(3, 0x0A, 0, 0, 1); // 显示记录数为0
			}
			break;
		case 0x04:
			if (state == 0x01) {
				uint8_t count = index_mode3 - 1; // 实际记录数
				if (count > 81)
					return;
				current_screen = 0x0005;
				if (connect_type == 0x04) {//wifi上传
				// 为了兼容老上位机，必须指定选择某项阻尼值
					if(filter_damping_num < 1 || filter_damping_num > 3){
						SetScreen(0x000C);
						return;
					}
					if(wifi_connect_state != 2) // 未连接wifi
					{
						SetScreen(0x0008);
						return;
					}
					wifi_transfer_type = 3;	
					Wifi_Start_TX();
				} else if (connect_type == 0x07) {//蓝牙上传
					ble_transfer_cancel = 1;
					BleUpload_Start(BLE_MODE_FORCED);
				}
			}
			break;
		default:
			break;
		}
	}
	if (screen_id == 0x06) // 参数设置页面
			{
		switch (control_id) {
		case 0x01:             // wifi开关
			if (state == 0x01) { // 开启
				SetButtonValue(6, 2, 0);
				SetButtonValue(7, 1, 0);
				AnimationPlayFrame(0, 0x07, 0);
				AnimationPlayFrame(1, 0x08, 0);
				AnimationPlayFrame(2, 0x0C, 0);
				AnimationPlayFrame(5, 0x08, 0);
				// 设置连接方式为wifi
				connect_type = 0x04;
				sfud_erase_write(g_flash, FLASH_CONNECT_ADDR,
						sizeof(connect_type), &connect_type);
			} else if (state == 0x00) {
				SetButtonValue(6, 1, 1);
			}
			break;
		case 0x02:             // 蓝牙开关
			if (state == 0x01) { // 开启
				SetButtonValue(6, 1, 0);
				SetButtonValue(4, 2, 0);
				AnimationPlayFrame(0, 0x07, 2);
				AnimationPlayFrame(1, 0x08, 1);
				AnimationPlayFrame(2, 0x0C, 1);
				AnimationPlayFrame(5, 0x08, 1);
				// 设置连接方式为蓝牙
				connect_type = 0x07;
				sfud_erase_write(g_flash, FLASH_CONNECT_ADDR,
						sizeof(connect_type), &connect_type);
			} else if (state == 0x00) {
				SetButtonValue(6, 2, 1);
			}
			break;
		case 0x07: // 阻尼1+
			if (state == 0x02) {
				damping1 = (damping1 + 10 > 350) ? 350 : damping1 + 10; // 长按步进10，防溢出
			} else if (state == 0x01) {
				if (damping1 < 350)
					damping1++;
			} else if (state == 0x00) { // 松开时写入 Flash
				if (damping1 > 350)
					damping1 = 350;
				sfud_erase_write(g_flash, FLASH_SAVE_ADDR + FLASH_DAMPING1_ADDR,
						sizeof(damping1), (uint8_t*) &damping1);
			}
			break;

		case 0x06: // 阻尼1-
			if (state == 0x02) {
				damping1 = (damping1 < 10) ? 0 : damping1 - 10; // 长按步进10，防下溢
			} else if (state == 0x01) {
				if (damping1 > 0)
					damping1--;
			} else if (state == 0x00) { // 松开时写入 Flash
				if (damping1 > 350)
					damping1 = 350;
				sfud_erase_write(g_flash, FLASH_SAVE_ADDR + FLASH_DAMPING1_ADDR,
						sizeof(damping1), (uint8_t*) &damping1);
			}
			break;
		case 0x09: // 阻尼2+
			if (state == 0x02) {
				damping2 = (damping2 + 10 > 350) ? 350 : damping2 + 10; // 长按步进10，防溢出
			} else if (state == 0x01) {
				if (damping2 < 350)
					damping2++;
			} else if (state == 0x00) { // 松开时写入 Flash
				if (damping2 > 350)
					damping2 = 350;
				sfud_erase_write(g_flash, FLASH_SAVE_ADDR + FLASH_DAMPING2_ADDR,
						sizeof(damping2), (uint8_t*) &damping2);
			}
			break;
		case 0x08: // 阻尼2-
			if (state == 0x02) {
				damping2 = (damping2 < 10) ? 0 : damping2 - 10; // 长按步进10，防下溢
			} else if (state == 0x01) {
				if (damping2 > 0)
					damping2--;
			} else if (state == 0x00) { // 松开时写入 Flash
				if (damping2 > 350)
					damping2 = 350;
				sfud_erase_write(g_flash, FLASH_SAVE_ADDR + FLASH_DAMPING2_ADDR,
						sizeof(damping2), (uint8_t*) &damping2);
			}
			break;
		case 0x0B: // 阻尼3+
			if (state == 0x02) {
				damping3 = (damping3 + 10 > 350) ? 350 : damping3 + 10; // 长按步进10，防溢出
			} else if (state == 0x01) {
				if (damping3 < 350)
					damping3++;
			} else if (state == 0x00) { // 松开时写入 Flash
				if (damping3 > 350)
					damping3 = 350;
				sfud_erase_write(g_flash, FLASH_SAVE_ADDR + FLASH_DAMPING3_ADDR,
						sizeof(damping3), (uint8_t*) &damping3);
			}
			break;

			break;
		case 0x0A: // 阻尼3-
			if (state == 0x02) {
				damping3 = (damping3 < 10) ? 0 : damping3 - 10; // 长按步进10，防下溢
			} else if (state == 0x01) {
				if (damping3 > 0)
					damping3--;
			} else if (state == 0x00) { // 松开时写入 Flash
				if (damping3 > 350)
					damping3 = 350;
				sfud_erase_write(g_flash, FLASH_SAVE_ADDR + FLASH_DAMPING3_ADDR,
						sizeof(damping3), (uint8_t*) &damping3);
			}
			break;
		default:
			break;
		}
	}
	if (screen_id == 0x07) // 蓝牙设置页
			{
		switch (control_id) {
		case 0x01: // 蓝牙开关
			if (state == 0x01) {
				stm32_to_esp32_data(ble_state, (uint8_t*) &ble_state_t[1], 1,
						esp32_stm32);
			} else {
				stm32_to_esp32_data(ble_state, (uint8_t*) &ble_state_t[0], 1,
						esp32_stm32);
			}
			break;
		default:
			break;
		}
	}
	if (screen_id == 0x0008) // wifi通讯失败
			{
		switch (control_id) {
		case 0x02: // 确认
			if (state == 0x01) {
				SetScreen(current_screen);
			}
			break;
		default:
			break;
		}
	}
	if (screen_id == 0x0009) // 蓝牙传输中页面
			{
		switch (control_id) {
		case 0x02: // 取消传输
			if (state == 0x01) {
				ble_transfer_cancel = 0;
				SetScreen(current_screen);
			}
			break;
		default:
			break;
		}
	}
	if (screen_id == 0x000A) // 点动档位选择
			{
		switch (control_id) {
		case 0x08: // 0.1
			if (state == 0x01) {
				jog_num = 1;
			}
			break;
		case 0x09: // 1
			if (state == 0x01) {
				jog_num = 9;
			}
			break;
		case 0x0A: // 5
			if (state == 0x01) {
				jog_num = 44;
			}
			break;
		case 0x0B: // 10
			if (state == 0x01) {
				jog_num = 89;
			}
			break;
		case 0x0C: // 15
			if (state == 0x01) {
				jog_num = 133;
			}
			break;
		case 0x0D: // 30
			if (state == 0x01) {
				jog_num = 267;
			}
			break;
		case 0x06: // 正向点动
			if (state == 0x01) {
				jog_msg[2] = 0x01;                  // 正向
				jog_msg[3] = (jog_num >> 8) & 0xFF; // jog_num 高字节
				jog_msg[4] = jog_num & 0xFF;        // jog_num 低字节
				HAL_UART_Transmit(&huart4, jog_msg, 6, 100);
			}
			break;

		case 0x07: // 反向点动
			if (state == 0x01) {
				jog_msg[2] = 0x00; // 反向
				jog_msg[3] = (jog_num >> 8) & 0xFF;
				jog_msg[4] = jog_num & 0xFF;
				HAL_UART_Transmit(&huart4, jog_msg, 6, 100);
			}
			break;
		case 0x05: // 关闭
			if (state == 0x01) {
				SetScreen(current_screen);
			}
			break;
		default:
			break;
		}
	}
	if (screen_id == 0x000B) // wifi上传成功页面
	{
		switch (control_id) {
		case 0x02: // 确认
			if (state == 0x01) {
				SetScreen(current_screen);
			}
			break;
		default:
			break;
		}
	}
	if(screen_id == 0x000C) // 强迫表格档位选择提示框
	{
		switch (control_id) {
		case 0x02: // 确认
			if (state == 0x01) {
				SetScreen(current_screen);
			}
			break;
		default:
			break;
		}
	}
}

// 文本输入
void NotifyText(uint16_t screen_id, uint16_t control_id, const char *str) {
	if (screen_id == 0x0000) // 首页
			{
		switch (control_id) {
		case 0x06: // 设备编号
			if (str == NULL) {
				break;
			}

			size_t len = strlen(str);
			if (len > 8) {
				// 长度不符合要求，拒绝保存，可按需添加错误提示
				break;
			}

			memset(device_id, 0, sizeof(device_id));
			memcpy(device_id, str, len);
			break;
		default:
			break;
		}
		return;
	}
	if (screen_id == 0x0004) // wifi设置页
			{
		switch (control_id) {
		case 0x03: // sid
			memset(wlan_sid, 0, sizeof(wlan_sid));
			memcpy(wlan_sid, str, strlen(str));
			stm32_to_esp32_data(wifi_sid_set, wlan_sid, strlen(str),
					esp32_stm32);
			break;
		case 0x04: // password
			memset(wlan_password, 0, sizeof(wlan_password));
			memcpy(wlan_password, str, strlen(str));
			stm32_to_esp32_data(wifi_password_set, wlan_password, strlen(str),
					esp32_stm32);
			break;
		case 0x05: // service IP
			memset(wlan_service_IP, 0, sizeof(wlan_service_IP));
			memcpy(wlan_service_IP, str, strlen(str));
			stm32_to_esp32_data(wifi_service_IP_set, wlan_service_IP,
					strlen(str), esp32_stm32);
			break;
		case 0x06: // service port
			memset(wlan_service_port, 0, sizeof(wlan_service_port));
			memcpy(wlan_service_port, str, strlen(str));
			stm32_to_esp32_data(wifi_service_port_set, wlan_service_port,
					strlen(str), esp32_stm32);
			break;
		default:
			break;
		}
	}
	if (screen_id == 0x0007) // 蓝牙设置页
			{
		switch (control_id) {
		case 0x03: // 名称
			memset(ble_name, 0, sizeof(ble_name));
			memcpy(ble_name, str, strlen(str));
			stm32_to_esp32_data(ble_name_set, ble_name, strlen(str),
					esp32_stm32);
			break;
		case 0x04: // MAC
			memset(ble_mac, 0, sizeof(ble_mac));
			memcpy(ble_mac, str, strlen(str));
			stm32_to_esp32_data(ble_mac_set, ble_mac, strlen(str), esp32_stm32);
			break;
		case 0x05: // 配对码
			memset(ble_paring, 0, sizeof(ble_paring));
			memcpy(ble_paring, str, strlen(str));
			stm32_to_esp32_data(ble_paring_set, ble_paring, strlen(str),
					esp32_stm32);
			break;
		default:
			break;
		}
	}
}

/*!
 *  \brief  进度条控件通知
 *  \details  通过GetControlValue时执行此函数
 *  \param screen_id 屏幕ID
 *  \param control_id 控件ID
 *  \param value 值
 */
void NotifyProgress(uint16_t screen_id, uint16_t control_id, uint32_t value) {
}

/*!
 *  \brief  滑块控件通知
 *  \details  获取滑块位置变化(通过GetControlValue)时执行此函数
 *  \param screen_id 屏幕ID
 *  \param control_id 控件ID
 *  \param value 值
 */
void NotifySlider(uint16_t screen_id, uint16_t control_id, uint32_t value) {
}

/*!
 *  \brief  仪表控件通知
 *  \details  通过GetControlValue时执行此函数
 *  \param screen_id 屏幕ID
 *  \param control_id 控件ID
 *  \param value 值
 */
void NotifyMeter(uint16_t screen_id, uint16_t control_id, uint32_t value) {
}

/*!
 *  \brief  菜单控件通知
 *  \details  当菜单项被按下或释放时执行此函数
 *  \param screen_id 屏幕ID
 *  \param control_id 控件ID
 *  \param item 菜单项索引
 *  \param state 按钮状态，0释放1按下
 */

void NotifyMenu(uint16_t screen_id, uint16_t control_id, uint8_t item,
		uint8_t state) {
	if (screen_id == 0x0002) {
		switch (control_id) {
		case 0x07: // 阻尼档位选择
			damping_num = item + 1;
			SendDampingCoefficient(damping_num);
			SetTextInt32(3, 8, damping_num, 0, 1);
			break;
		}
	}
	if (screen_id == 0x0005) { // 受控表格页面-筛选
		switch (control_id) {
		case 0x09: // 阻尼档位选择
		{
			filter_damping_num = item;
			// 筛选
			FilterData(item);
		}
			break;
		}
	}
}

/*!
 *  \brief  选择器控件通知
 *  \details  当选择器控件值被写入时触发
 *  \param screen_id 屏幕ID
 *  \param control_id 控件ID
 *  \param item 当前选择项
 */
void NotifySelector(uint16_t screen_id, uint16_t control_id, uint8_t item) {
}

/*!
 *  \brief  记录按钮点击通知
 *  \param screen_id 屏幕ID
 *  \param control_id 控件ID
 *  \param line 选中行
 */
void NotifyRecord(uint16_t screen_id, uint16_t control_id, uint8_t line) {
	if (screen_id == 0x0005 && control_id == 0x01) { // 受迫振荡表格
		line_flag = line;
	}
}

/*!
 *  \brief  实时时间控件时间通知函数
 *  \param screen_id 屏幕ID
 *  \param control_id 控件ID
 */
void NotifyTimer(uint16_t screen_id, uint16_t control_id) {
}

/*!
 *  \brief  获取内部FLASH状态回调
 *  \param status 0失败，1成功
 *  \param _data 数据指针
 *  \param length 数据长度
 */
void NotifyReadFlash(uint8_t status, uint8_t *_data, uint16_t length) {
}

/*!
 *  \brief  写内部FLASH状态回调
 *  \param status 0失败，1成功
 */
void NotifyWriteFlash(uint8_t status) {
}

/*!
 *  \brief  获取RTC时间，注意返回的是BCD码
 *  \param year 年（BCD码）
 *  \param month 月（BCD码）
 *  \param week 星期（BCD码）
 *  \param day 日（BCD码）
 *  \param hour 时（BCD码）
 *  \param minute 分（BCD码）
 *  \param second 秒（BCD码）
 */
void NotifyReadRTC(uint8_t year, uint8_t month, uint8_t week, uint8_t day,
		uint8_t hour, uint8_t minute, uint8_t second) {
}

/*!
 *  \brief  触摸屏任务回调
 *  \param status 0失败，1成功
 */
qsize size = 0;
uint8_t cmd_buffer[CMD_MAX_SIZE];
void touch_hmi_task_callback(void) {
	size = queue_find_cmd(cmd_buffer, CMD_MAX_SIZE);
	if (size > 0 && cmd_buffer[1] != 0x07) // 串口屏指令处
			{
		ProcessMessage((PCTRL_MSG) cmd_buffer, size);
	}
	if (hmi_updata_flag) {
		hmi_updata_flag = 0;
		UpdateUI();
	}
}

/*!
 *  \brief  获取flash数据
 */
void GetFlashData(void) {
	sfud_read(g_flash, FLASH_SAVE_ADDR + FLASH_DAMPING1_ADDR, 2,
			(uint8_t*) &damping1);
	sfud_read(g_flash, FLASH_SAVE_ADDR + FLASH_DAMPING2_ADDR, 2,
			(uint8_t*) &damping2);
	sfud_read(g_flash, FLASH_SAVE_ADDR + FLASH_DAMPING3_ADDR, 2,
			(uint8_t*) &damping3);
	// 读取连接方式
	sfud_read(g_flash, FLASH_CONNECT_ADDR, sizeof(connect_type), &connect_type);
	if (connect_type == 0x04) {
		SetButtonValue(6, 1, 1);
		AnimationPlayFrame(0, 0x07, 0);
		AnimationPlayFrame(1, 0x08, 0);
		AnimationPlayFrame(2, 0x0C, 0);
		AnimationPlayFrame(5, 0x08, 0);
	} else if (connect_type == 0x07) {
		SetButtonValue(6, 2, 1);
		AnimationPlayFrame(0, 0x07, 2);
		AnimationPlayFrame(1, 0x08, 1);
		AnimationPlayFrame(2, 0x0C, 1);
		AnimationPlayFrame(5, 0x08, 1);
	}
}

/*
 * brief  根据阻尼档发送对应系数
 *  \param item 阻尼档位
 */
void SendDampingCoefficient(uint8_t item) {
	if (item == 1) {
		target_current_msg[2] = (uint8_t) (damping1 >> 8);
		target_current_msg[3] = (uint8_t) (damping1 & 0xFF);
		HAL_UART_Transmit(&huart4, target_current_msg, 6, 100);
		return;
	} else if (item == 2) {
		target_current_msg[2] = (uint8_t) (damping2 >> 8);
		target_current_msg[3] = (uint8_t) (damping2 & 0xFF);
		HAL_UART_Transmit(&huart4, target_current_msg, 6, 100);
		return;
	} else if (item == 3) {
		target_current_msg[2] = (uint8_t) (damping3 >> 8);
		target_current_msg[3] = (uint8_t) (damping3 & 0xFF);
		HAL_UART_Transmit(&huart4, target_current_msg, 6, 100);
		return;
	}
}

/*!
 *  \brief  获取配置信息
 */
void GetConfigInfo(void) {
	uint8_t config_info[1] = { 0 };
	stm32_to_esp32_data(get_config_info, config_info, 1, esp32_stm32);
}

/*!
 *  \brief  档位筛选
 *  \param target_damping 目标阻尼档位
 *  \details  根据目标阻尼档位筛选数据，展示全部数据或指定阻尼档位的数据
 */
void FilterData(uint8_t target_damping) {
	// 展示全部数据
	if (target_damping == 0) {
		// 清空表格，重新填充筛选结果
		Record_Clear(5, 1);
		for (uint32_t i = 0; i < index_mode3 - 1; i++) {
			snprintf((char*) output_str, sizeof(output_str),
					"%lu;%.1f;%.4f;%.4f;%.1f;%.1f", forced_record_buf[i].index,
					forced_record_buf[i].amplitude,
					forced_record_buf[i].drive_period,
					forced_record_buf[i].period,
					forced_record_buf[i].phase_diff1,
					forced_record_buf[i].phase_diff2);
			Record_Add(5, 1, output_str);
		}
		return;
	}
	// 筛选
	index_filtered = 0;
	for (uint32_t i = 0; i < index_mode3 - 1; i++) {
		if (forced_record_buf[i].damping == target_damping) {
			filtered_record_buf[index_filtered] = forced_record_buf[i];
			// filtered_record_buf[index_filtered].index = index_filtered; // 重新编号
			index_filtered++;
		}
	}

	// 清空表格，重新填充筛选结果
	Record_Clear(5, 1);
	for (uint32_t i = 0; i < index_filtered; i++) {
		snprintf((char*) output_str, sizeof(output_str),
				"%lu;%.1f;%.4f;%.4f;%.1f;%.1f", filtered_record_buf[i].index,
				filtered_record_buf[i].amplitude,
				filtered_record_buf[i].drive_period,
				filtered_record_buf[i].period,
				filtered_record_buf[i].phase_diff1,
				filtered_record_buf[i].phase_diff2);
		Record_Add(5, 1, output_str);
	}
}

/*!
 *  \brief  根据阻尼档位展示表格数据----阻尼振荡
 *          仅在档位切换或有新数据入库时才实际刷新表格
 *  \param num 阻尼档位
 */
// void FilterByDampedNum(uint8_t num) {
//     if (current_screen != 0x0002) {
//         return;
//     }

//     if (num == last_damping_num && index_mode2 == last_index_mode2) {
//         return;
//     }

//     Record_Clear(2, 1);

//     uint32_t local_index = 0;
//     for (uint32_t i = 0; i < index_mode2 - 1; i++) {
//         if (damped_record_buf[i].damping == num) {
//             local_index++;
//             snprintf((char *)output_str, sizeof(output_str), "%lu;%.1f;%.4f",
//                      local_index, damped_record_buf[i].amplitude, damped_record_buf[i].period);
//             Record_Add(2, 1, output_str);
//         }
//     }

//     if (local_index > 0) {
//         biaoge_bottom(2, 1, local_index - 1);
//     }

//     last_damping_num = num;
//     last_index_mode2 = index_mode2;
// }

void FilterByDampedNum(uint8_t num) {
    if (current_screen != 0x0002) {
        return;
    }

    if (num == last_damping_num && index_mode2 == last_index_mode2) {
        return; // 档位没变，数据也没变，什么都不用做
    }

    if (num != last_damping_num) {
        // 场景一：档位切换，必须整表重建
        Record_Clear(2, 1);

        uint32_t local_index = 0;
        for (uint32_t i = 0; i < index_mode2 - 1; i++) {
            if (damped_record_buf[i].damping == num) {
                local_index++;
                snprintf((char *)output_str, sizeof(output_str), "%lu;%.1f;%.4f",
                         local_index, damped_record_buf[i].amplitude, damped_record_buf[i].period);
                Record_Add(2, 1, output_str);
            }
        }

        if (local_index > 0) {
            biaoge_bottom(2, 1, local_index - 1);
        }

        last_local_index = local_index;

    } else {
        // 场景二：档位没变，只是有新数据入库，增量追加即可，不清空
        uint32_t local_index = last_local_index;

        // 只扫描上次处理位置之后新增的那一段数据
        for (uint32_t i = last_index_mode2 - 1; i < index_mode2 - 1; i++) {
            if (damped_record_buf[i].damping == num) {
                local_index++;
                snprintf((char *)output_str, sizeof(output_str), "%lu;%.1f;%.4f",
                         local_index, damped_record_buf[i].amplitude, damped_record_buf[i].period);
                Record_Add(2, 1, output_str);  // 只追加，不clear
            }
        }

        if (local_index > last_local_index) {
            biaoge_bottom(2, 1, local_index - 1);
        }

        last_local_index = local_index;
    }

    last_damping_num = num;
    last_index_mode2 = index_mode2;
}
