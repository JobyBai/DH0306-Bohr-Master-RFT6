/*
 * cmd_queue.h
 *
 *  Created on: Mar 19, 2025
 *      Author: hzdh
 */

#ifndef BSP_CMD_QUEUE_CMD_QUEUE_H_
#define BSP_CMD_QUEUE_CMD_QUEUE_H_

#include "hmi_driver.h"

typedef unsigned char qdata;
typedef unsigned short qsize;

/*!
 *  \brief  数据处理函数
 */
void queue_reset(void);

/*!
 * \brief  数据处理函数
 * \detail 用于处理接收到的数据，通过回调函数进行进一步处理
 * \param  _data 指向数据的指针
 */
void queue_push(qdata _data);

/*!
 *  \brief  从指令队列中获取下一条指令
 *  \param  cmd 指令队列的指针
 *  \param  buf_len 指令队列缓冲区的大小
 *  \return 指令长度，0 表示没有更多指令
 */
qsize queue_find_cmd(qdata *cmd,qsize buf_len);

#endif /* BSP_CMD_QUEUE_CMD_QUEUE_H_ */
