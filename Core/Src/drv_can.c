#include <drv_can.h>
#include "can.h"

//////////////////////////////////////////////////////////////////////////////////	 

#define     DRV_CAN_MAX_STD_ID      0x8FF   // 标准帧ID上限
#define     DRV_CAN_MAX_FRAMES      20      // 数据帧缓存上限
#define     DRV_CAN_MAX_CALLBACKS   20      // 函数指针上限

// 数据帧
typedef struct {
    uint32_t ID;
    uint8_t buf[8];
    uint8_t len;
} CAN_Frame;

// 数据帧队列
typedef struct {
    CAN_Frame data[DRV_CAN_MAX_FRAMES]; // 队列数据
    uint8_t head; // 队首
    uint8_t tail; // 队尾
} CAN_FrameQueue;

static void CAN_QueuePush(CAN_FrameQueue* queue, CAN_Frame frame);  // 队列中添加一个元素
static void CAN_QueuePop(CAN_FrameQueue* queue, CAN_Frame* frame);  // 队列中取出一个元素
static uint8_t CAN_QueueGetNum(CAN_FrameQueue* queue);              // 返回队列中元素数量

static CAN_FrameQueue canQueue = {{},0,0}; // 接收数据帧队列
static DRV_CAN_RxCallback canRxCallback_table[DRV_CAN_MAX_CALLBACKS]; // 回调函数指针表
static uint8_t canRxCallback_count = 0; // 回调函数计数



// CAN启动发送与接收
void DRV_CAN_Start(void)
{
    HAL_CAN_Start(&hcan);   // 启动can传输
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); // 启动CAN接收
}

// CAN过滤器初始化  注：过滤器ID使用左对齐(MSB)
void DRV_CAN_InitFilter(uint32_t mode, uint32_t scale, uint32_t id_h, uint32_t mask_id_h, uint32_t id_l, uint32_t mask_id_l)
{
    CAN_FilterTypeDef sFilterCfg;
    sFilterCfg.FilterActivation = CAN_FILTER_ENABLE;    // 过滤器开启
    sFilterCfg.FilterBank = 0;                          // 第0组
    sFilterCfg.FilterMode = mode;
    sFilterCfg.FilterScale = scale;
    sFilterCfg.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 过滤器0关联到FIFO0
    sFilterCfg.FilterIdHigh     = id_h;
    sFilterCfg.FilterMaskIdHigh = mask_id_h;
    sFilterCfg.FilterIdLow      = id_l;
    sFilterCfg.FilterMaskIdLow  = mask_id_l;

    HAL_CAN_ConfigFilter(&hcan, &sFilterCfg);           // 初始化过滤器
}

// CAN过滤器初始化 List16位模式  注：ID使用左对齐(MSB)
void DRV_CAN_InitFilterList16(uint32_t id_h, uint32_t mask_id_h, uint32_t id_l, uint32_t mask_id_l)
{
    CAN_FilterTypeDef sFilterCfg;
    sFilterCfg.FilterActivation = CAN_FILTER_ENABLE;    // 过滤器开启
    sFilterCfg.FilterBank = 0;                          // 第0组
    sFilterCfg.FilterMode = CAN_FILTERMODE_IDLIST;      // 使用列表方式
    sFilterCfg.FilterScale = CAN_FILTERSCALE_16BIT;     // 16位模式
    sFilterCfg.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 过滤器0关联到FIFO0
    sFilterCfg.FilterIdHigh     = id_h;
    sFilterCfg.FilterMaskIdHigh = mask_id_h;
    sFilterCfg.FilterIdLow      = id_l;
    sFilterCfg.FilterMaskIdLow  = mask_id_l;

    HAL_CAN_ConfigFilter(&hcan, &sFilterCfg);           // 初始化过滤器
}

// CAN过滤器初始化 Mask16位模式  注：ID使用左对齐(MSB)
void DRV_CAN_InitFilterMask16(uint32_t id_h, uint32_t mask_id_h, uint32_t id_l, uint32_t mask_id_l)
{
    CAN_FilterTypeDef sFilterCfg;
    sFilterCfg.FilterActivation = CAN_FILTER_ENABLE;    // 过滤器开启
    sFilterCfg.FilterBank = 0;                          // 第0组
    sFilterCfg.FilterMode = CAN_FILTERMODE_IDMASK;      // 使用掩码方式
    sFilterCfg.FilterScale = CAN_FILTERSCALE_16BIT;     // 16位模式
    sFilterCfg.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 过滤器0关联到FIFO0
    sFilterCfg.FilterIdHigh     = id_h;
    sFilterCfg.FilterMaskIdHigh = mask_id_h;
    sFilterCfg.FilterIdLow      = id_l;
    sFilterCfg.FilterMaskIdLow  = mask_id_l;
    
    HAL_CAN_ConfigFilter(&hcan, &sFilterCfg);           // 初始化过滤器
}

// CAN发送帧数据
uint8_t DRV_CAN_SendData(uint32_t ID, uint8_t* buf, uint8_t len)
{
    CAN_TxHeaderTypeDef txHeader;
    uint32_t mailBoxID = 0;

    if (len > 8) len = 8;
    if (ID <= DRV_CAN_MAX_STD_ID) // 为标准帧
    {
        txHeader.StdId = ID;        // StdID
        txHeader.IDE = CAN_ID_STD;  // 标准帧
    }
    else // 为扩展帧
    {
        txHeader.ExtId = ID;        // StdID
        txHeader.IDE = CAN_ID_EXT;  // 扩展帧
    }
    txHeader.RTR = CAN_RTR_DATA;    // 数据帧
    txHeader.DLC = len;             // 数据长度
    txHeader.TransmitGlobalTime = DISABLE; // 不发送时间戳

    // 发送数据帧，返回邮箱号
    return HAL_CAN_AddTxMessage(&hcan, &txHeader, buf, &mailBoxID);
}

// CAN处理数据
void DRV_CAN_ProcessData(void)
{
    CAN_Frame frame;
    while (CAN_QueueGetNum(&canQueue)) // 非空状态
    {
        CAN_QueuePop(&canQueue, &frame); // 取出一帧

        for (uint8_t i = 0; i < canRxCallback_count; i++) {
            if (canRxCallback_table[i] != NULL) {
                canRxCallback_table[i](frame.ID, frame.buf, frame.len);
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////
// 注册CAN接收处理回调函数
void DRV_RegisterCanRxCallback(DRV_CAN_RxCallback callback)
{
    if (callback == NULL) return;

    if (canRxCallback_count < DRV_CAN_MAX_CALLBACKS) {
        canRxCallback_table[canRxCallback_count] = callback;
        canRxCallback_count++;
    }
}

// 注销CAN接收处理回调函数
void DRV_UnregisterCanRxCallback(DRV_CAN_RxCallback callback)
{
    if (callback == NULL) return;

    for (uint8_t i = 0; i < canRxCallback_count; i++) {
        if (canRxCallback_table[i] == callback) {
            // 移除该项，将后面的项前移
            for (uint8_t j = i; j < canRxCallback_count - 1; j++) {
                canRxCallback_table[j] = canRxCallback_table[j + 1];
            }
            canRxCallback_count--;
            break;
        }
    }
}

// 统一的CAN接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *m_hcan)
{
    if (m_hcan != &hcan) return;

    CAN_Frame frame;
    CAN_RxHeaderTypeDef rxHeader;
    if (HAL_CAN_GetRxMessage(m_hcan, CAN_RX_FIFO0, &rxHeader, frame.buf) != HAL_OK) return; // 读取接收到的消息

    if (rxHeader.IDE == CAN_ID_STD) // 标准帧
        frame.ID = rxHeader.StdId;
    else // 扩展帧
        frame.ID = rxHeader.ExtId;

    if (rxHeader.RTR == CAN_RTR_DATA) // 数据帧
        frame.len = rxHeader.DLC;

    CAN_QueuePush(&canQueue, frame); // 插入一帧
}

// 队列中添加一个元素
static void CAN_QueuePush(CAN_FrameQueue* queue, CAN_Frame frame)
{
    if (queue == NULL) return;

    uint8_t pos = (queue->tail + 1) % DRV_CAN_MAX_FRAMES;
    if (pos != queue->tail) // 非满状态
    {
        queue->data[queue->tail] = frame; // 从队尾插入帧
        queue->tail = pos; // 队尾自增
    }
}

// 队列中取出一个元素
static void CAN_QueuePop(CAN_FrameQueue* queue, CAN_Frame* frame)
{
    if (queue == NULL) return;

    if (queue->head != queue->tail) // 非满状态
    {
        *frame = queue->data[queue->head]; // 从队首取出帧
        queue->head = (queue->head + 1) % DRV_CAN_MAX_FRAMES; // 队首自增
    }
}

// 返回队列中元素数量
static uint8_t CAN_QueueGetNum(CAN_FrameQueue* queue)
{
    if (queue == NULL) return 0;

    return (DRV_CAN_MAX_FRAMES + queue->tail - queue->head) % DRV_CAN_MAX_FRAMES;
}

