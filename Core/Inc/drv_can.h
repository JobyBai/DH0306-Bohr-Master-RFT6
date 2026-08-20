#ifndef __DRV_CAN_H
#define __DRV_CAN_H
//#include "sys.h"

#include "stm32f1xx_hal.h"
//////////////////////////////////////////////////////////////////////////////////	 

typedef void (*DRV_CAN_RxCallback)(uint32_t ID, uint8_t* buf, uint8_t len); // 数据接收回调函数指针

void DRV_CAN_Start(void); // CAN启动发送与接收
void DRV_CAN_InitFilter(uint32_t mode, uint32_t scale, uint32_t id_h, uint32_t mask_id_h, uint32_t id_l, uint32_t mask_id_l); // CAN过滤器初始化
void DRV_CAN_InitFilterList16(uint32_t id_h, uint32_t mask_id_h, uint32_t id_l, uint32_t mask_id_l); // CAN过滤器初始化 List16位模式
void DRV_CAN_InitFilterMask16(uint32_t id_h, uint32_t mask_id_h, uint32_t id_l, uint32_t mask_id_l); // CAN过滤器初始化 Mask16位模式


uint8_t DRV_CAN_SendData(uint32_t ID, uint8_t* buf, uint8_t len);    // CAN发送数据
void DRV_CAN_ProcessData(void);  // CAN处理数据
void DRV_RegisterCanRxCallback(DRV_CAN_RxCallback callback); // 注册CAN接收处理回调函数
void DRV_UnregisterCanRxCallback(DRV_CAN_RxCallback callback); // 注销CAN接收处理回调函数

#endif
