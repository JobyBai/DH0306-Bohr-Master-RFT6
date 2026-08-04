/*
 * ESP32.c
 *
 *  Created on: May 23, 2025
 *      Author: dhyf02
 */


#include "esp32.h"
#include "usart.h"
#include "string.h"
#include "hmi_control.h"
#include "ble_table_upload.h"
#include "wifi_upload.h"


typedef struct Y_QUEUE
{
    y_qsize y_head;                                                       //队列头
    y_qsize y_tail;                                                       //队列尾
    y_qsize y_data[Y_QUEUE_MAX_SIZE];                                       //队列数据缓存区
}YQUEUE;

static YQUEUE y_que = {0,0,{0}};                                            //指令队列
static uint32_t y_cmd_state = 0;                                           //队列帧尾检测状态
static y_qsize y_cmd_pos = 0;                                              //当前指令指针位置

y_qsize y_size = 0;
uint8_t y_cmd_buffer[Y_CMD_MAX_SIZE];

/*!
*  \brief  清空指令数据
*/
void y_queue_reset(void)
{
	y_que.y_head = y_que.y_tail = 0;
	y_cmd_pos = y_cmd_state = 0;
}
/*!
* \brief  添加指令数据
* \detial 串口接收的数据，通过此函数放入指令队列
*  \param  _data 指令数据
*/
void y_queue_push(y_qdata y_data)
{
	y_qsize y_pos = (y_que.y_head+1)%Y_QUEUE_MAX_SIZE;
    if(y_pos!=y_que.y_tail)                                                //非满状态
    {
    	y_que.y_data[y_que.y_head] = y_data;
    	y_que.y_head = y_pos;
    }
}

//从队列中取一个数据
static void y_queue_pop(y_qdata*  y_data)
{
    if(y_que.y_tail!=y_que.y_head)                                          //非空状态
    {
        *y_data = y_que.y_data[y_que.y_tail];
        y_que.y_tail = (y_que.y_tail+1)%Y_QUEUE_MAX_SIZE;
    }
}

//获取队列中有效数据个数
static y_qsize y_queue_size()
{
    return ((y_que.y_head+Y_QUEUE_MAX_SIZE-y_que.y_tail)%Y_QUEUE_MAX_SIZE);
}


/**
 * 获取当前系统时间(毫秒)
 * @return 当前系统时间(毫秒)
 */
static uint32_t get_current_time_ms(void)
{
    return HAL_GetTick();
}
/*!
*  \brief  从指令队列中取出一条完整的指令（基于超时检测，无帧头帧尾）
*  \param  buffer 指令接收缓存区
*  \param  buf_len 指令接收缓存区大小
*  \param  timeout_ms 超时时间（毫秒）
*  \return  指令长度，0表示队列中无完整指令或超时
*/
y_qsize y_queue_find_cmd(y_qdata *buffer, y_qsize buf_len)
{
    static uint32_t last_data_time = 0;  // 上次接收到数据的时间戳
    y_qsize y_cmd_size = 0;
    y_qdata y_data = 0;

    // 获取当前时间（需要根据平台实现）
    uint32_t current_time = get_current_time_ms();

    // 检查是否超时
    if (y_cmd_pos > 0 && (current_time - last_data_time) > 1)
    {
        // 超时，返回当前已接收的指令长度
        y_cmd_size = y_cmd_pos;
        y_cmd_pos = 0;
        return y_cmd_size;
    }

    while (y_queue_size() > 0)
    {
        // 取一个数据
        y_queue_pop(&y_data);
        last_data_time = current_time;  // 更新最后接收时间

        // 防止缓冲区溢出
        if (y_cmd_pos >= buf_len)
        {
            y_cmd_size = y_cmd_pos;    // 缓冲区溢出，返回当前已接收的指令长度
            y_cmd_pos = 0;
            return y_cmd_size;
        }

        // 存入数据
        buffer[y_cmd_pos++] = y_data;
    }
    return 0;  // 没有形成完整的一帧
}


y_qsize y_queue_find_cmd2(y_qdata *buffer, y_qsize buf_len)
{
    //static uint32_t last_data_time = 0;
    y_qsize y_cmd_size = 0;
    y_qdata y_data = 0;

	static uint16_t dataAllLength,n;
	uint8_t sum_low;

    while(y_queue_size() > 0)
    {
        //取一个数据
        y_queue_pop(&y_data);
        if(y_data!=0xEE&&y_cmd_pos==0)
		{
			continue;
		}
		if(y_data!=0xBE&&y_cmd_pos==1)
		{
			y_cmd_pos=0;
		}
		if(y_cmd_pos==3)
		{
			n=y_data;
			dataAllLength=n+6; //数据长度n+帧头2+功能码1+数字长度+校验码+连接码
		}

        //放置缓冲区溢出
        if(y_cmd_pos <buf_len)
        {
            buffer[y_cmd_pos++] = y_data;
        }
		if(y_cmd_pos==dataAllLength)
		{
			y_cmd_size=y_cmd_pos;
			y_cmd_pos=0;
			sum_low=sum_low_8bits(buffer,n+4);
			if(sum_low!=buffer[n+4])
			{
				y_cmd_pos=0;
				return 0;
			}
			return y_cmd_size;
		}
    }
    return 0;
}

void ESP32_ack_data(uint8_t *p ,uint16_t len)
{
	HAL_UART_Transmit(&huart2, p, len, 1000);
}



uint8_t sum_low_8bits(uint8_t *numbers, uint16_t count) {
    uint32_t total = 0;
    for (uint16_t i = 0; i < count; i++) {
        total += numbers[i];
    }
    return (uint8_t)(total & 0xFF);
}


uint8_t temp_data[64]={0};
void stm32_to_esp32_data(cmd_code_t cmd_code,uint8_t *p,uint16_t len,link_code_t link_code)
{
	if(len<60)
	{
		temp_data[0]=0xEE;
		temp_data[1]=0xBE;
		temp_data[2]=cmd_code;
		temp_data[3]=len;
		memcpy((uint8_t *)&temp_data[4],p,len);
		temp_data[len+4]=sum_low_8bits((uint8_t *)&temp_data,len+4);
		temp_data[len+5]=link_code;
		ESP32_ack_data(temp_data,len+6);
	}
}

/*!
*  \brief  检查wifi是否正常连接
*/
void check_wifi_connect_state(void)
{
     uint8_t wifi_connect_state_resp[1] = {0x01};
	 stm32_to_esp32_data(wifi_connect_state_set, wifi_connect_state_resp, 1, esp32_stm32);
}

/*!
 * 
*  \brief  处理ESP32发来的指令
*  \return  无
*/
uint8_t wifi_state_flag=0;

void ESP32_232_data_tx(uint8_t *data,uint16_t len)
{
	if(data[len-1]==0x10)
	{
        if(data[0]==0xEE&&data[1]==0xBE)
        {
        	if(data[2]==wifi_sid_set)memcpy(wlan_sid,&data[4],data[3]);
        	if(data[2]==wifi_password_set)memcpy(wlan_password,&data[4],data[3]);
        	if(data[2]==wifi_service_IP_set)memcpy(wlan_service_IP,&data[4],data[3]);
        	if(data[2]==wifi_service_port_set)memcpy(wlan_service_port,&data[4],data[3]);
        	if(data[2]==wifi_state)wifi_state_flag=1;
             if(data[2]==wifi_connect_state_set){
                wifi_connect_state = data[4];				
                if(wifi_connect_state == 0)
                {
                    SetScreen(0x0008);
                }
             }
            if(data[2]==ble_name_set)memcpy(ble_name,&data[4],data[3]);
            if(data[2]==ble_mac_set)memcpy(ble_mac,&data[4],data[3]);
            if(data[2]==ble_paring_set)memcpy(ble_paring,&data[4],data[3]);
            // 处理wifi上传指令
            if(data[2]==wifi_upload){
                // WifiUpload_Process(data, sizeof(data));
            }
        }
        
	}
}


uint8_t ble_msg_receive_OK[]={0x6F,0x6B,0x0D,0x0A,0x12};//蓝牙发送成功
uint8_t ble_msg_receive_ERROR[]={0x65,0x72,0x72,0x6F,0x72,0x0D,0x0A,0x12};//蓝牙发送失败
// uint8_t wifi_msg_receive_OK[]={0x6F,0x6B,0x0D,0x0A,0x13};//wifi发送成功
// uint8_t wifi_msg_receive_ERROR[]={0x65,0x72,0x72,0x6F,0x72,0x0D,0x0A,0x13};//wifi发送失败
 void ESP32_232_communication_task_callback(void)
 {
	  y_size = y_queue_find_cmd2(y_cmd_buffer,Y_CMD_MAX_SIZE);
	  if(y_size>6)//正常指令
	    {
	    	ESP32_232_data_tx(y_cmd_buffer,y_size);
	    	memset(y_cmd_buffer,0,y_size);
	    }else{
        if(uart2_rxcomplete_flag == 1)
        {
            uart2_rxcomplete_flag = 0 ;
            if(memcmp(uart2_rx_data,ble_msg_receive_OK,5) == 0){// 蓝牙发送成功
                 BleUpload_OnAck(1);
            }
            // else if(memcmp(uart2_rx_data,wifi_msg_receive_OK,5) == 0){// wifi发送成功
            //     //  SetScreen(0x000B);// 展示wifi发送成功
            //     WifiUpload_OnAck(1);
            // }
            else if(memcmp(uart2_rx_data,ble_msg_receive_ERROR,8) == 0){// 蓝牙发送失败
                BleUpload_OnAck(0);
            }
            // else if(memcmp(uart2_rx_data,wifi_msg_receive_ERROR,8) == 0){// wifi发送失败
            //     WifiUpload_OnAck(0);
            // }
        }
    }

 }




