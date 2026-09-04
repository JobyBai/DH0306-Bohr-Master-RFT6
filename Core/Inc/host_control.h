#ifndef INC_HOST_CONTROL_H_
#define INC_HOST_CONTROL_H_


#include "main.h"

#define H_CMD_HEAD_1 0XEE                                                  //帧头1
#define H_CMD_HEAD_2 0XBE                                                  // 帧头2       
// #define H_CMD_TAIL 0X55                                            //帧尾
#define H_QUEUE_MAX_SIZE 256   /*!< 指令接收缓冲区大小，根据需要调整，尽量设置大一些*/
#define H_CMD_MAX_SIZE  256

typedef unsigned char h_qdata;
typedef unsigned short h_qsize;


extern uint16_t collect_frequency; // 采集频率
extern uint8_t collect_flag; // 采集标志 0:关闭 1:开启



void host_communication_task_callback(void);
void h_queue_push(h_qdata h_data);
void h_queue_reset(void);
void collect_data_task_callback(void);

#endif /* INC_HOST_CONTROL_H_ */
