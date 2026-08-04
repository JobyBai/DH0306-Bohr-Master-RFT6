#ifndef INC_POWER_CONTROL_H_
#define INC_POWER_CONTROL_H_


#include "main.h"

#define P_CMD_HEAD 0XAA                                                  //帧头
#define P_CMD_TAIL 0X55                                            //帧尾
#define P_QUEUE_MAX_SIZE 256   /*!< 指令接收缓冲区大小，根据需要调整，尽量设置大一些*/
#define P_CMD_MAX_SIZE  256

typedef unsigned char p_qdata;
typedef unsigned short p_qsize;




void power_send_data(uint8_t *p ,uint16_t len);
void power_communication_task_callback(void);
void p_queue_push(p_qdata p_data);
void p_queue_reset(void);

#endif /* INC_POWER_CONTROL_H_ */
