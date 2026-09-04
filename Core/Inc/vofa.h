#ifndef VOFFA_H
#define VOFFA_H

#include"stdint.h"
#include"string.h"
#include"usart.h"
/*宏定义*/
#define data_num 2  //发送数量
#define data_len (data_num * 4 + 4)  //发送长度

typedef struct
{
  float Send_Data_Array[data_num];  //发送数据数组
  uint8_t Byte_Data_Array[data_len];  //发送数据
} vofa_data_t;
//全局变量
extern vofa_data_t VofaSend;



//函数声明

 void VofaSend_Task(uint16_t adc_value, uint16_t encoder_value);
 void VofaData_Process(void);    

#endif