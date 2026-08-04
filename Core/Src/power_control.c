#include "power_control.h"
#include "usart.h"
#include "string.h"
#include "hmi_control.h"


typedef struct P_QUEUE
{
    p_qsize p_head;                                                       //队列头
    p_qsize p_tail;                                                       //队列尾
    p_qsize p_data[P_QUEUE_MAX_SIZE];                                       //队列数据缓存区
}PQUEUE;

static PQUEUE p_que = {0,0,{0}};                                            //指令队列
static uint32_t p_cmd_state = 0;                                           //队列帧尾检测状态
static p_qsize p_cmd_pos = 0;                                              //当前指令指针位置

p_qsize p_size = 0;
uint8_t p_cmd_buffer[P_CMD_MAX_SIZE];

/*!
*  \brief  清空指令数据
*/
void p_queue_reset(void)
{
	p_que.p_head = p_que.p_tail = 0;
	p_cmd_pos = p_cmd_state = 0;
}
/*!
* \brief  添加指令数据
* \detial 串口接收的数据，通过此函数放入指令队列
*  \param  _data 指令数据
*/
void p_queue_push(p_qdata p_data)
{
	p_qsize p_pos = (p_que.p_head+1)%P_QUEUE_MAX_SIZE;
    if(p_pos!=p_que.p_tail)                                                //非满状态
    {
    	p_que.p_data[p_que.p_head] = p_data;
    	p_que.p_head = p_pos;
    }
}

//从队列中取一个数据
static void p_queue_pop(p_qdata*  p_data)
{
    if(p_que.p_tail!=p_que.p_head)                                          //非空状态
    {
        *p_data = p_que.p_data[p_que.p_tail];
        p_que.p_tail = (p_que.p_tail+1)%P_QUEUE_MAX_SIZE;
    }
}

//获取队列中有效数据个数
static p_qsize p_queue_size()
{
    return ((p_que.p_head+P_QUEUE_MAX_SIZE-p_que.p_tail)%P_QUEUE_MAX_SIZE);
}


/*!
*  \brief  从指令队列中获取到一条完整指令指针
*  \param  cmd 指令接收缓冲区
*  \param  buf_len 指令接收缓冲区大小
*  \return  指令长度，0表示没有获取到完整指令
*/
p_qsize p_queue_find_cmd(p_qdata *buffer,p_qsize buf_len)
{
    p_qsize cmd_size = 0;
    p_qdata _data = 0;

    while(p_queue_size()>0)
    {
        //取一个数据
        p_queue_pop(&_data);

        if(p_cmd_pos==0&&_data!=P_CMD_HEAD)                               //指令第一个字节必须是帧头，否则丢弃
        {
            continue;
        }
        //    LED2_ON;
        if(p_cmd_pos<buf_len)                                           //防止缓冲区溢出
            buffer[p_cmd_pos++] = _data;

        p_cmd_state = ((p_cmd_state<<8)|_data);                           //拼接最近4个字节，组成一个32位数

        //最近4个字节与帧尾匹配，得到完整帧
        if(p_cmd_state==P_CMD_TAIL)
        {
            //LED2_ON;
            cmd_size = p_cmd_pos;                                       //指令字节长度
            p_cmd_state = 0;                                            //重新检测帧尾
            p_cmd_pos = 0;                                              //复位指针指向
            return cmd_size;
        }
    }
    return 0;                                                         //没有形成完整的一帧
}


// 处理从机返回指令，判断是否发送成功
void power_data_deal(uint8_t *data,uint16_t len)
{
    
    switch(data[1]){
        case 0x08:  // 切换继电器位置：消磁成功
	        HAL_UART_Transmit(&huart4, demagnetize_open_msg, 6, 100);// 开启消磁
            break;
        case 0x09:  // 切换继电器位置：恒流源 				
			SendDampingCoefficient(damping_num);// 发送阻尼系数
            break;
        default:
            break;
    }
}




 void power_communication_task_callback(void)
 {
	  p_size = p_queue_find_cmd(p_cmd_buffer,P_CMD_MAX_SIZE);
	  if(p_size > 2 && p_cmd_buffer[0]==0xAA)
	      {
	    	  power_data_deal(p_cmd_buffer,p_size);
	    	  memset(p_cmd_buffer,0,p_size);
	      }
 }



