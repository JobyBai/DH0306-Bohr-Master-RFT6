#include "vofa.h"


 vofa_data_t VofaSend;

/**
 * @brief 发送任务
 * 
 */
 void VofaSend_Task(uint16_t adc_value, uint16_t encoder_value)
{
    VofaSend.Send_Data_Array[0] = adc_value * 1.0f;
    VofaSend.Send_Data_Array[1] = encoder_value * 1.0f;
	
	VofaData_Process();
}
/**
 * @brief vofa数据处理
 * 
 * 
 */
void VofaData_Process(void)
{//将float数据转换为单字节数据
    memcpy(VofaSend.Byte_Data_Array, VofaSend.Send_Data_Array, data_len);
    
    VofaSend.Byte_Data_Array[data_len - 4] = 0x00;
    VofaSend.Byte_Data_Array[data_len - 3] = 0x00;
    VofaSend.Byte_Data_Array[data_len - 2] = 0x80;
    VofaSend.Byte_Data_Array[data_len - 1] = 0x7f;
    HAL_UART_Transmit_DMA(&huart1, VofaSend.Byte_Data_Array, data_len);
}