/*
 * cmd_queue.c
 *
 *  Created on: Mar 19, 2025
 *      Author: hzdh
 */
#include "cmd_queue.h"

#define CMD_HEAD 0XEE                                                  //帧头
#define CMD_TAIL 0XFFFCFFFF                                            //帧尾

typedef struct _QUEUE
{
    qsize _head;                                                       //队列头
    qsize _tail;                                                       //队列尾
    qdata _data[QUEUE_MAX_SIZE];                                       //队列数据缓冲区
}QUEUE;

static QUEUE que = {0,0,{0}};                                            //指令队列
static uint32_t cmd_state = 0;                                           //检测帧尾的状态
static qsize cmd_pos = 0;                                              //当前指针指向位置

/*!
*  \brief  重置指令队列
*/
void queue_reset()
{
    que._head = que._tail = 0;
    cmd_pos = cmd_state = 0;
}
/*!
* \brief  指令队列压入
* \detial 用于接收到的数据，通过此函数压入指令队列
*  \param  _data 指令数据
*/
void queue_push(qdata _data)
{
    qsize pos = (que._head+1)%QUEUE_MAX_SIZE;
    if(pos!=que._tail)                                                //非满状态
    {
        que._data[que._head] = _data;
        que._head = pos;
    }
}

//从队列取出一个数据
static void queue_pop(qdata* _data)
{
    if(que._tail!=que._head)                                          //非空状态
    {
        *_data = que._data[que._tail];
        que._tail = (que._tail+1)%QUEUE_MAX_SIZE;
    }
}

//获取队列中有效数据个数
static qsize queue_size()
{
    return ((que._head+QUEUE_MAX_SIZE-que._tail)%QUEUE_MAX_SIZE);
}
/*!
*  \brief  从指令队列中获取到一条完整指令指针
*  \param  cmd 指令接收缓冲区
*  \param  buf_len 指令接收缓冲区大小
*  \return  指令长度，0表示没有获取到完整指令
*/
qsize queue_find_cmd(qdata *buffer,qsize buf_len)
{
    qsize cmd_size = 0;
    qdata _data = 0;

    while(queue_size()>0)
    {
        //取一个数据
        queue_pop(&_data);

        if(cmd_pos==0&&_data!=CMD_HEAD)                               //指令第一个字节必须是帧头，否则丢弃
        {
            continue;
        }
        //    LED2_ON;
        if(cmd_pos<buf_len)                                           //防止缓冲区溢出
            buffer[cmd_pos++] = _data;

        cmd_state = ((cmd_state<<8)|_data);                           //拼接最近4个字节，组成一个32位数

        //最近4个字节与帧尾匹配，得到完整帧
        if(cmd_state==CMD_TAIL)
        {
            //LED2_ON;
            cmd_size = cmd_pos;                                       //指令字节长度
            cmd_state = 0;                                            //重新检测帧尾
            cmd_pos = 0;                                              //复位指针指向

#if(CRC16_ENABLE)
            //去掉指令头尾EE和尾FFFCFFFF共5个字节，只对数据部分CRC
            if(!CheckCRC16(buffer+1,cmd_size-5))                      //CRC校验
                return 0;

            cmd_size -= 2;                                            //去掉CRC16的2个字节，
#endif
            return cmd_size;
        }
    }
    return 0;                                                         //没有形成完整的一帧
}
