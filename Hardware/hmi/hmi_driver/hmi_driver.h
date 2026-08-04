/*
 * hmi_driver.h
 *
 *  Created on: Mar 19, 2025
 *      Author: hzdh
 */

#ifndef BSP_HMI_DRIVER_HMI_DRIVER_H_
#define BSP_HMI_DRIVER_HMI_DRIVER_H_

#include "../../Core/Inc/usart.h"

#define FIRMWARE_VER 917   /*!< ���ִ˹̼��汾������ʵ��Ļһ�£�ȷ���������ܿ���*/
#define CRC16_ENABLE 0         /*!< �����ҪCRC16У�鹦�ܣ��޸Ĵ˺�Ϊ1(��ʱ��Ҫ��VisualTFT��������CRCУ��)*/
#define CRC16_ENABLE_232 0
#define CMD_MAX_SIZE 64        /*!<����ָ���С��������Ҫ�������������ô�һЩ*/
#define QUEUE_MAX_SIZE 128   /*!< ָ����ջ�������С��������Ҫ�������������ô�һЩ*/

#define SD_FILE_EN 0
#define TX_8(P1) SEND_DATA((P1)&0xFF)                    //���͵����ֽ�
#define TX_8N(P,N) SendNU8((uint8_t *)P,N)                 //����N���ֽ�
#define TX_16(P1) TX_8((P1)>>8);TX_8(P1)                 //����16λ����
#define TX_16N(P,N) SendNU16((uint16_t *)P,N)              //����N��16λ����
#define TX_32(P1) TX_16((P1)>>16);TX_16((P1)&0xFFFF)     //����32λ����

#define TX_8_232(P1) SEND_DATA_232((P1)&0xFF)                    //���͵����ֽ�
#define TX_16_232(P1) TX_8_232((P1)>>8);TX_8_232(P1)                 //����16λ����
#define TX_32_232(P1) TX_16_232((P1)>>16);TX_16_232((P1)&0xFFFF)     //����32λ����

#define SEND_DATA_232(P) SendChar_232(P)          //����һ���ֽ�
#define BEGIN_CMD_232() TX_8_232(0XEE)            //֡ͷ
#define END_CMD_232() TX_16_232(0XFFEF)       //֡β
#define SEND_DATA(P) SendChar(P)          //����һ���ֽ�
#define BEGIN_CMD() TX_8(0XEE)            //֡ͷ
#define END_CMD() TX_32(0XFFFCFFFF)       //֡β



#include "usart.h"
/*!
 *  \brief  ��������Ƿ����CRC16У��
 *  \param buffer ��У������ݣ�ĩβ�洢CRC16
 *  \param n ���ݳ��ȣ�����CRC16
 *  \return У��ͨ������1�����򷵻�0
 */
uint16_t CheckCRC16(uint8_t *buffer,uint16_t n);

void GUI_FcolorConDots(uint16_t *dot,uint16_t dot_cnt);
/*!
 *  \brief  ��ʱ
 *  \param  n ��ʱʱ��(���뵥λ)
 */
void DelayMS(unsigned int n);

/*!
 *  \brief  �����豸���ã�����֮����Ҫ�����������޸Ĳ����ʡ���������������������ʽ
 */
void LockDeviceConfig(void);

/*!
 *  \brief  �����豸����
 */
void UnlockDeviceConfig(void);

/*!
 *  \brief     �޸Ĵ������Ĳ�����
 *  \details  ������ѡ�Χ[0~14]����Ӧʵ�ʲ�����
                   {1200,2400,4800,9600,19200,38400,57600,115200,1000000,2000000,218750,437500,875000,921800,2500000}
 *  \param  option ������ѡ��
 */
void SetCommBps(uint8_t option);

/*!
 *  \brief  ������������
 */
void SetHandShake(void);

/*!
 *  \brief  ����ǰ��ɫ
 *  \param  color ǰ��ɫ
 */
void SetFcolor(uint16_t color);

/*!
 *  \brief  ���ñ���ɫ
 *  \param  color ����ɫ
 */
void SetBcolor(uint16_t color);

/*!
 *  \brief  �������
 */
void GUI_CleanScreen();

/*!
 *  \brief  �������ּ��
 *  \param  x_w ������
  *  \param  y_w ������
 */
void SetTextSpace(uint8_t x_w, uint8_t y_w);

/*!
 *  \brief  ����������ʾ����
 *  \param  enable �Ƿ���������
 *  \param  width ���
 *  \param  height �߶�
 */
void SetFont_Region(uint8_t enable,uint16_t width,uint16_t height );

/*!
 *  \brief  ���ù���ɫ
 *  \param  fillcolor_dwon ��ɫ�½�
 *  \param  fillcolor_up ��ɫ�Ͻ�
 */
void SetFilterColor(uint16_t fillcolor_dwon, uint16_t fillcolor_up);

/*!
 *  \brief  ���ù���ɫ
 *  \param  x λ��X����
 *  \param  y λ��Y����
 *  \param  back ��ɫ�Ͻ�
 *  \param  font ����
 *  \param  strings �ַ�������
 */
void DisText(uint16_t x, uint16_t y,uint8_t back,uint8_t font,unsigned char *strings );

/*!
 *  \brief    ��ʾ���
 *  \param  enable �Ƿ���ʾ
 *  \param  x λ��X����
 *  \param  y λ��Y����
 *  \param  width ���
 *  \param  height �߶�
 */
void DisCursor(uint8_t enable,uint16_t x, uint16_t y,uint8_t width,uint8_t height );

/*!
 *  \brief      ��ʾȫ��ͼƬ
 *  \param  image_id ͼƬ����
 *  \param  masken �Ƿ�����͸������
 */
void DisFull_Image(uint16_t image_id,uint8_t masken);

/*!
 *  \brief      ָ��λ����ʾͼƬ
 *  \param  x λ��X����
 *  \param  y λ��Y����
 *  \param  image_id ͼƬ����
 *  \param  masken �Ƿ�����͸������
 */
void DisArea_Image(uint16_t x,uint16_t y,uint16_t image_id,uint8_t masken);

/*!
 *  \brief      ��ʾ�ü�ͼƬ
 *  \param  x λ��X����
 *  \param  y λ��Y����
 *  \param  image_id ͼƬ����
 *  \param  image_x ͼƬ�ü�λ��X����
 *  \param  image_y ͼƬ�ü�λ��Y����
 *  \param  image_l ͼƬ�ü�����
 *  \param  image_w ͼƬ�ü��߶�
 *  \param  masken �Ƿ�����͸������
 */
void DisCut_Image(uint16_t x,uint16_t y,uint16_t image_id,uint16_t image_x,uint16_t image_y,
                   uint16_t image_l, uint16_t image_w,uint8_t masken);

/*!
 *  \brief      ��ʾGIF����
 *  \param  x λ��X����
 *  \param  y λ��Y����
 *  \param  flashimage_id ͼƬ����
 *  \param  enable �Ƿ���ʾ
 *  \param  playnum ���Ŵ���
 */
void DisFlashImage(uint16_t x,uint16_t y,uint16_t flashimage_id,uint8_t enable,uint8_t playnum);

/*!
 *  \brief      ����
 *  \param  x λ��X����
 *  \param  y λ��Y����
 */
void GUI_Dot(uint16_t x,uint16_t y);

/*!
 *  \brief      ����
 *  \param  x0 ��ʼλ��X����
 *  \param  y0 ��ʼλ��Y����
 *  \param  x1 ����λ��X����
 *  \param  y1 ����λ��Y����
 */
void GUI_Line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/*!
 *  \brief      ������
 *  \param  mode ģʽ
 *  \param  dot ���ݵ�
 *  \param  dot_cnt ����
 */
void GUI_ConDots(uint8_t mode,uint16_t *dot,uint16_t dot_cnt);

/*!
 *  \brief      ������Բ
 *  \param  x0 Բ��λ��X����
 *  \param  y0 Բ��λ��Y����
 *  \param  r �뾶
 */
void GUI_Circle(uint16_t x0, uint16_t y0, uint16_t r);

/*!
*  \brief  切换画面时自动清楚图层
*  \param  screen_id 画面ID
*  \param
*/
void GUI_Clean(uint16_t Enable);
void GUI_ConSpaceDots(uint16_t x,uint16_t x_space,uint16_t *dot_y,uint16_t dot_cnt);
/*!
 *  \brief      ��ʵ��Բ
 *  \param  x0 Բ��λ��X����
 *  \param  y0 Բ��λ��Y����
 *  \param  r �뾶
 */
void GUI_CircleFill(uint16_t x0, uint16_t y0, uint16_t r);

/*!
 *  \brief      ������
 *  \param  x0 Բ��λ��X����
 *  \param  y0 Բ��λ��Y����
 *  \param  r �뾶
 *  \param  sa ��ʼ�Ƕ�
 *  \param  ea ��ֹ�Ƕ�
 */
void GUI_Arc(uint16_t x,uint16_t y, uint16_t r,uint16_t sa, uint16_t ea);

/*!
 *  \brief      �����ľ���
 *  \param  x0 ��ʼλ��X����
 *  \param  y0 ��ʼλ��Y����
 *  \param  x1 ����λ��X����
 *  \param  y1 ����λ��Y����
 */
void GUI_Rectangle(uint16_t x0, uint16_t y0, uint16_t x1,uint16_t y1 );

/*!
 *  \brief      ��ʵ�ľ���
 *  \param  x0 ��ʼλ��X����
 *  \param  y0 ��ʼλ��Y����
 *  \param  x1 ����λ��X����
 *  \param  y1 ����λ��Y����
 */
void GUI_RectangleFill(uint16_t x0, uint16_t y0, uint16_t x1,uint16_t y1 );

/*!
 *  \brief      ��������Բ
 *  \param  x0 ��ʼλ��X����
 *  \param  y0 ��ʼλ��Y����
 *  \param  x1 ����λ��X����
 *  \param  y1 ����λ��Y����
 */
void GUI_Ellipse (uint16_t x0, uint16_t y0, uint16_t x1,uint16_t y1 );

/*!
 *  \brief      ��ʵ����Բ
 *  \param  x0 ��ʼλ��X����
 *  \param  y0 ��ʼλ��Y����
 *  \param  x1 ����λ��X����
 *  \param  y1 ����λ��Y����
 */
void GUI_EllipseFill (uint16_t x0, uint16_t y0, uint16_t x1,uint16_t y1 );

/*!
 *  \brief      ����
 *  \param  x0 ��ʼλ��X����
 *  \param  y0 ��ʼλ��Y����
 *  \param  x1 ����λ��X����
 *  \param  y1 ����λ��Y����
 */
void SetBackLight(uint8_t light_level);

/*!
 *  \brief   ����������
 *  \time  time ����ʱ��(���뵥λ)
 */
void SetBuzzer(uint8_t time);

/*!
 *  \brief   ����������
 *  \param enable ����ʹ��
 *  \param beep_on ����������
 *  \param work_mode ��������ģʽ��0���¾��ϴ���1�ɿ����ϴ���2�����ϴ�����ֵ��3���º��ɿ����ϴ�����
 *  \param press_calibration �������������20��У׼��������0���ã�1����
 */
void SetTouchPaneOption(uint8_t enbale,uint8_t beep_on,uint8_t work_mode,uint8_t press_calibration);

/*!
 *  \brief   У׼������
 */
void	CalibrateTouchPane();

/*!
 *  \brief  ����������
 */
void TestTouchPane();

/*!
 *  \brief      ���õ�ǰд��ͼ��
 *  \details  һ������ʵ��˫����Ч��(��ͼʱ������˸)��
 *  \details  uint8_t layer = 0;
 *  \details  WriteLayer(layer);    //����д���
 *  \details  ClearLayer(layer);    //ʹͼ���͸��
 *  \details  //���һϵ�л�ͼָ��
 *  \details  //DisText(100,100,0,4,"hello hmi!!!");
 *  \details  DisplyLayer(layer);  //�л���ʾ��
 *  \details  layer = (layer+1)%2;  //˫�����л�
 *  \see DisplyLayer
 *  \see ClearLayer
 *  \param  layer ͼ����
 */
void WriteLayer(uint8_t layer);

/*!
 *  \brief      ���õ�ǰ��ʾͼ��
 *  \param  layer ͼ����
 */
void DisplyLayer(uint8_t layer);

/*!
 *  \brief      ���ͼ�㣬ʹͼ����͸��
 *  \param  layer ͼ����
 */
void ClearLayer(uint8_t layer);

/*!
 *  \brief  д���ݵ��������û��洢��
 *  \param  startAddress ��ʼ��ַ
 *  \param  length �ֽ���
 *  \param  _data ��д�������
 */
void WriteUserFlash(uint32_t startAddress,uint16_t length,uint8_t *_data);

/*!
 *  \brief  �Ӵ������û��洢����ȡ����
 *  \param  startAddress ��ʼ��ַ
 *  \param  length �ֽ���
 */
void ReadUserFlash(uint32_t startAddress,uint16_t length);

/*!
 *  \brief      ����ͼ��
 *  \param  src_layer ԭʼͼ��
 *  \param  dest_layer Ŀ��ͼ��
 */
void CopyLayer(uint8_t src_layer,uint8_t dest_layer);

/*!
 *  \brief      ���õ�ǰ����
 *  \param  screen_id ����ID
 */
void SetScreen(uint16_t screen_id);

/*!
 *  \brief      ��ȡ��ǰ����
 */
void GetScreen(uint16_t screen_id);

/*!
 *  \brief     ����\���û������
 *  \details ����\����һ��ɶ�ʹ�ã����ڱ�����˸�����ˢ���ٶ�
 *  \details �÷���
 *	\details SetScreenUpdateEnable(0);//��ֹ����
 *	\details һϵ�и��»����ָ��
 *	\details SetScreenUpdateEnable(1);//��������
 *  \param  enable 0���ã�1����
 */
void SetScreenUpdateEnable(uint8_t enable);

/*!
 *  \brief     ���ÿؼ����뽹��
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  focus �Ƿ�������뽹��
 */
void SetControlFocus(uint16_t screen_id,uint16_t control_id,uint8_t focus);

/*!
 *  \brief     ��ʾ\���ؿؼ�
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  visible �Ƿ���ʾ
 */
void SetControlVisiable(uint16_t screen_id,uint16_t control_id,uint8_t visible);

/*!
 *  \brief     ���ô����ؼ�ʹ��
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  enable �ؼ��Ƿ�ʹ��
 */
void SetControlEnable(uint16_t screen_id,uint16_t control_id,uint8_t enable);

/*!
 *  \brief     ��ȡ�ؼ�ֵ
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void GetControlValue(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief     ���ð�ť״̬
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  value ��ť״̬
 */
void SetButtonValue(uint16_t screen_id,uint16_t control_id,unsigned char value);

/*!
 *  \brief     �����ı�ֵ
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  str �ı�ֵ
 */
void SetTextValue(uint16_t screen_id,uint16_t control_id,unsigned char *str);

#if FIRMWARE_VER>=908

/*!
 *  \brief     �����ı�Ϊ����ֵ��Ҫ��FIRMWARE_VER>=908
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  value �ı���ֵ
 *  \param  sign 0-�޷��ţ�1-�з���
 *  \param  fill_zero ����λ��������ʱ��ಹ��
 */
void SetTextInt32(uint16_t screen_id,uint16_t control_id,uint32_t value,uint8_t sign,uint8_t fill_zero);

/*!
 *  \brief     �����ı������ȸ���ֵ��Ҫ��FIRMWARE_VER>=908
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  value �ı���ֵ
 *  \param  precision С��λ��
 *  \param  show_zeros Ϊ1ʱ����ʾĩβ0
 */
void SetTextFloat(uint16_t screen_id,uint16_t control_id,float value,uint8_t precision,uint8_t show_zeros);

#endif

/*!
 *  \brief      ���ý���ֵ
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  value ��ֵ
 */
void SetProgressValue(uint16_t screen_id,uint16_t control_id,uint32_t value);

/*!
 *  \brief     �����Ǳ�ֵ
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  value ��ֵ
 */
void SetMeterValue(uint16_t screen_id,uint16_t control_id,uint32_t value);

/*!
 *  \brief      ���û�����
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  value ��ֵ
 */
void SetSliderValue(uint16_t screen_id,uint16_t control_id,uint32_t value);

/*!
 *  \brief      ����ѡ��ؼ�
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  item ��ǰѡ��
 */
void SetSelectorValue(uint16_t screen_id,uint16_t control_id,uint8_t item);

/*!
 *  \brief      ��ʼ���Ŷ���
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void AnimationStart(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief      ֹͣ���Ŷ���
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void AnimationStop(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief      ��ͣ���Ŷ���
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void AnimationPause(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief     �����ƶ�֡
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  frame_id ֡ID
 */
void AnimationPlayFrame(uint16_t screen_id,uint16_t control_id,uint8_t frame_id);

/*!
 *  \brief     ������һ֡
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void AnimationPlayPrev(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief     ������һ֡
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void AnimationPlayNext(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief     ���߿ؼ�-���ͨ��
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  channel ͨ����
 *  \param  color ��ɫ
 */
void GraphChannelAdd(uint16_t screen_id,uint16_t control_id,uint8_t channel,uint16_t color);

/*!
 *  \brief     ���߿ؼ�-ɾ��ͨ��
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  channel ͨ����
 */
void GraphChannelDel(uint16_t screen_id,uint16_t control_id,uint8_t channel);

/*!
 *  \brief     ���߿ؼ�-�������
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  channel ͨ����
 *  \param  pData ��������
 *  \param  nDataLen ���ݸ���
 */
void GraphChannelDataAdd(uint16_t screen_id,uint16_t control_id,uint8_t channel,uint8_t *pData,uint16_t nDataLen);

/*!
 *  \brief     ���߿ؼ�-�������
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  channel ͨ����
 */
void GraphChannelDataClear(uint16_t screen_id,uint16_t control_id,uint8_t channel);

/*!
 *  \brief     ���߿ؼ�-������ͼ����
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  x_offset ˮƽƫ��
 *  \param  x_mul ˮƽ����ϵ��
 *  \param  y_offset ��ֱƫ��
 *  \param  y_mul ��ֱ����ϵ��
 */
void GraphSetViewport(uint16_t screen_id,uint16_t control_id,int16_t x_offset,uint16_t x_mul,int16_t y_offset,uint16_t y_mul);

/*!
 *  \brief     ��ʼ��������
 *  \param  screen_id ����ID
 */
void BatchBegin(uint16_t screen_id);

/*!
 *  \brief     �������°�ť�ؼ�
 *  \param  control_id �ؼ�ID
 *  \param  value ��ֵ
 */
void BatchSetButtonValue(uint16_t control_id,uint8_t state);

/*!
 *  \brief     �������½������ؼ�
 *  \param  control_id �ؼ�ID
 *  \param  value ��ֵ
 */
void BatchSetProgressValue(uint16_t control_id,uint32_t value);

/*!
 *  \brief     �������»������ؼ�
 *  \param  control_id �ؼ�ID
 *  \param  value ��ֵ
 */
void BatchSetSliderValue(uint16_t control_id,uint32_t value);

/*!
 *  \brief     ���������Ǳ�ؼ�
 *  \param  control_id �ؼ�ID
 *  \param  value ��ֵ
 */
void BatchSetMeterValue(uint16_t control_id,uint32_t value);

/*!
 *  \brief     ���������ı��ؼ�
 *  \param  control_id �ؼ�ID
 *  \param  strings �ַ���
 */
void BatchSetText(uint16_t control_id,unsigned char *strings);

/*!
 *  \brief     �������¶���\ͼ��ؼ�
 *  \param  control_id �ؼ�ID
 *  \param  frame_id ֡ID
 */
void BatchSetFrame(uint16_t control_id,uint16_t frame_id);

#if FIRMWARE_VER>=908

/*!
 *  \brief     �������ÿؼ��ɼ�
 *  \param  control_id �ؼ�ID
 *  \param  visible ֡ID
 */
void BatchSetVisible(uint16_t control_id,uint8_t visible);

/*!
 *  \brief     �������ÿؼ�ʹ��
 *  \param  control_id �ؼ�ID
 *  \param  enable ֡ID
 */
void BatchSetEnable(uint16_t control_id,uint8_t enable);

#endif

/*!
 *  \brief    ������������
 */
void BatchEnd();

/*!
 *  \brief     ���õ���ʱ�ؼ�
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  timeout ����ʱ(��)
 */
void SeTimer(uint16_t screen_id,uint16_t control_id,uint32_t timeout);

/*!
 *  \brief     ��������ʱ�ؼ�
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void StartTimer(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief     ֹͣ����ʱ�ؼ�
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void StopTimer(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief     ��ͣ����ʱ�ؼ�
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void PauseTimer(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief     ���ÿؼ�����ɫ
 *  \details  ֧�ֿؼ������������ı�
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  color ����ɫ
 */
void SetControlBackColor(uint16_t screen_id,uint16_t control_id,uint16_t color);

/*!
 *  \brief     ���ÿؼ�ǰ��ɫ
  * \details  ֧�ֿؼ���������
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  color ǰ��ɫ
 */
void SetControlForeColor(uint16_t screen_id,uint16_t control_id,uint16_t color);

/*!
 *  \brief     ��ʾ\���ص����˵��ؼ�
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  show �Ƿ���ʾ��Ϊ0ʱfocus_control_id��Ч
 *  \param  focus_control_id �������ı��ؼ�(�˵��ؼ�������������ı��ؼ�)
 */
void ShowPopupMenu(uint16_t screen_id,uint16_t control_id,uint8_t show,uint16_t focus_control_id);

/*!
 *  \brief     ��ʾ\����ϵͳ����
 *  \param  show 0���أ�1��ʾ
 *  \param  x ������ʾλ��X����
 *  \param  y ������ʾλ��Y����
 *  \param  type 0С���̣�1ȫ����
 *  \param  option 0�����ַ���1���룬2ʱ������
 *  \param  max_len ����¼���ַ���������
 */
void ShowKeyboard(uint8_t show,uint16_t x,uint16_t y,uint8_t type,uint8_t option,uint8_t max_len);

#if FIRMWARE_VER>=914
/*!
 *  \brief     ����������
 *  \param  ui_lang �û���������0~9
 *  \param  sys_lang ϵͳ��������-0���ģ�1Ӣ��
 */
void SetLanguage(uint8_t ui_lang,uint8_t sys_lang);
#endif

#if FIRMWARE_VER>=917
/*!
 *  \brief     ��ʼ����ؼ���ֵ��FLASH
 *  \param  version ���ݰ汾�ţ�������ָ������16λΪ���汾�ţ���16λΪ�ΰ汾��
 *  \param  address �������û��洢���Ĵ�ŵ�ַ��ע���ֹ��ַ�ص�����ͻ
 */
void FlashBeginSaveControl(uint32_t version,uint32_t address);

/*!
 *  \brief     ����ĳ���ؼ�����ֵ��FLASH
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void FlashSaveControl(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief     �������
 */
void FlashEndSaveControl();

/*!
 *  \brief     ��FLASH�лָ��ؼ�����
 *  \param  version ���ݰ汾�ţ����汾�ű�����洢ʱһ�£���������ʧ��
 *  \param  address �������û��洢���Ĵ�ŵ�ַ
 */
void FlashRestoreControl(uint32_t version,uint32_t address);
#endif

#if FIRMWARE_VER>=921
/*!
 *  \brief     ������ʷ���߲�������ֵ(���ֽڣ�uint8_t��int8)
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  value ����������
 *  \param  channel ͨ����
 */
void HistoryGraph_SetValueInt8(uint16_t screen_id,uint16_t control_id,uint8_t *value,uint8_t channel);

/*!
 *  \brief     ������ʷ���߲�������ֵ(˫�ֽڣ�uint16_t��int16)
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  value ����������
 *  \param  channel ͨ����
 */
void HistoryGraph_SetValueInt16(uint16_t screen_id,uint16_t control_id,uint16_t *value,uint8_t channel);

/*!
 *  \brief     ������ʷ���߲�������ֵ(���ֽڣ�uint32_t��int32)
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  value ����������
 *  \param  channel ͨ����
 */
void HistoryGraph_SetValueInt32(uint16_t screen_id,uint16_t control_id,uint32_t *value,uint8_t channel);

/*!
 *  \brief     ������ʷ���߲�������ֵ(�����ȸ�����)
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  value ����������
 *  \param  channel ͨ����
 */
void HistoryGraph_SetValueFloat(uint16_t screen_id,uint16_t control_id,float *value,uint8_t channel);

/*!
 *  \brief     ������ֹ��ʷ���߲���
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  enable 0-��ֹ��1-����
 */
void HistoryGraph_EnableSampling(uint16_t screen_id,uint16_t control_id,uint8_t enable);

/*!
 *  \brief     ��ʾ��������ʷ����ͨ��
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  channel ͨ�����
 *  \param  show 0-���أ�1-��ʾ
 */
void HistoryGraph_ShowChannel(uint16_t screen_id,uint16_t control_id,uint8_t channel,uint8_t show);


/*!
 *  \brief     ������ʷ����ʱ�䳤��(����������)
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  sample_count һ����ʾ�Ĳ�������
 */
void HistoryGraph_SetTimeLength(uint16_t screen_id,uint16_t control_id,uint16_t sample_count);

/*!
 *  \brief     ��ʷ�������ŵ�ȫ��
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void HistoryGraph_SetTimeFullScreen(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief     ������ʷ�������ű���ϵ��
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  zoom ���Űٷֱ�(zoom>100%ʱˮƽ������С�������Ŵ�)
 *  \param  max_zoom �������ƣ�һ�������ʾ��������
  *  \param  min_zoom �������ƣ�һ��������ʾ��������
 */
void HistoryGraph_SetTimeZoom(uint16_t screen_id,uint16_t control_id,uint16_t zoom,uint16_t max_zoom,uint16_t min_zoom);
#endif

#if SD_FILE_EN
/*!
 *  \brief     ���SD���Ƿ����
 */
void SD_IsInsert(void);

#define	FA_READ				        0x01  /*!< �ɶ�ȡ*/
#define	FA_WRITE			            0x02  /*!< ��д��*/
#define	FA_CREATE_NEW		    0x04  /*!< �������ļ�������ļ��Ѿ����ڣ��򷵻�ʧ��*/
#define	FA_CREATE_ALWAYS	0x08  /*!< �������ļ�������ļ��Ѿ����ڣ��򸲸�*/
#define	FA_OPEN_EXISTING	    0x00  /*!< ���ļ�������ļ������ڣ��򷵻�ʧ��*/
#define	FA_OPEN_ALWAYS		0x10  /*!< ���ļ�������ļ������ڣ��򴴽����ļ�*/

/*!
 *  \brief     �򿪻򴴽��ļ�
 *  \param  filename �ļ�����(��ASCII����)
 *  \param  mode ģʽ����ѡ���ģʽ����FA_XXXX
 */
void SD_CreateFile(uint8_t *filename,uint8_t mode);

/*!
 *  \brief     �Ե�ǰʱ�䴴���ļ�������:20161015083000.txt
 *  \param  ext �ļ���׺������ txt
 */
void SD_CreateFileByTime(uint8_t *ext);

/*!
 *  \brief     �ڵ�ǰ�ļ�ĩβд������
 *  \param  buffer ����
 *  \param  dlc ���ݳ���
 */
void SD_WriteFile(uint8_t *buffer,uint16_t dlc);

/*!
 *  \brief     ��ȡ��ǰ�ļ�
 *  \param  offset �ļ�λ��ƫ��
 *  \param  dlc ���ݳ���
 */
void SD_ReadFile(uint32_t offset,uint16_t dlc);

/*!
 *  \brief     ��ȡ��ǰ�ļ�����
 */
void SD_GetFileSize();

/*!
 *  \brief     �رյ�ǰ�ļ�
 */
void SD_CloseFile();
#endif

/*!
 *  \brief     ��¼�ؼ�-��������
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  value �澯ֵ
 *  \param  time �澯������ʱ�䣬Ϊ0ʱʹ����Ļ�ڲ�ʱ��
 */
void Record_SetEvent(uint16_t screen_id,uint16_t control_id,uint16_t value,uint8_t *time);

/*!
 *  \brief     ��¼�ؼ�-�������
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  value �澯ֵ
 *  \param  time �澯�����ʱ�䣬Ϊ0ʱʹ����Ļ�ڲ�ʱ��
 */
void Record_ResetEvent(uint16_t screen_id,uint16_t control_id,uint16_t value,uint8_t *time);

/*!
 *  \brief    ��¼�ؼ�- ��ӳ����¼
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  record һ����¼(�ַ���)������ͨ���ֺŸ��������磺��һ��;�ڶ���;������;
 */
void Record_Add(uint16_t screen_id,uint16_t control_id,uint8_t *record);
void Record_Add123(uint16_t screen_id,uint16_t control_id,float value);
void Record_Add2(uint16_t screen_id,uint16_t control_id,uint8_t *record,uint8_t *record1,uint8_t *record2,uint8_t *record3,uint8_t *record4);
/*!
 *  \brief     ��¼�ؼ�-�����¼����
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void Record_Clear(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief     ��¼�ؼ�-���ü�¼��ʾƫ��
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  offset ��ʾƫ�ƣ�������λ��
 */
void Record_SetOffset(uint16_t screen_id,uint16_t control_id,uint16_t offset);

/*!
 *  \brief     ��¼�ؼ�-��ȡ��ǰ��¼��Ŀ
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 */
void Record_GetCount(uint16_t screen_id,uint16_t control_id);

/*!
 *  \brief     ��¼�ؼ�-��ȡָ��λ�õļ�¼
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  index ����
 */
void Record_Read(uint16_t screen_id,uint16_t control_id,uint16_t index);

/*!
 *  \brief     ��¼�ؼ�-�޸�ָ��λ�õļ�¼
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  index ����
 *  \param  record һ����¼(�ַ���)������ͨ���ֺŸ��������磺��һ��;�ڶ���;������;
 */
void Record_Modify(uint16_t screen_id,uint16_t control_id,uint16_t index,uint8_t *record);

/*!
 *  \brief     ��¼�ؼ�-ɾ��ָ��λ�õļ�¼
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  index ����
 */
void Record_Delete(uint16_t screen_id,uint16_t control_id,uint16_t index);

/*!
 *  \brief     ��¼�ؼ�-��ָ��λ�ò����¼
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  index ����
 *  \param  record һ����¼(�ַ���)������ͨ���ֺŸ��������磺��һ��;�ڶ���;������;
 */
void Record_Insert(uint16_t screen_id,uint16_t control_id,uint16_t index,uint8_t *record);

/*!
 *  \brief     ��¼�ؼ�-ѡ��ָ����¼����
 *  \param  screen_id ����ID
 *  \param  control_id �ؼ�ID
 *  \param  index ����
 */
void Record_Select(uint16_t screen_id,uint16_t control_id,uint16_t index);

struct LINE_STRUCT{

uint16_t QUXIAN_L_UP_X ; //�����������Ͻ�X����
uint16_t QUXIAN_L_UP_Y ; //�����������Ͻ�Y����
uint16_t QUXIAN_R_DOWN_X ; //�����������½�X����
uint16_t QUXIAN_R_DOWN_Y ; //�����������½�Y����

int16_t  max_X;//X�������ֵ
int16_t  max_Y;//Y�������ֵ
int16_t  min_X;//X������Сֵ
int16_t  min_Y;//Y������Сֵ

};
void Line1_init(struct LINE_STRUCT *p);
void Line2_init(struct LINE_STRUCT *p);
void GUI_FcolorConDots1(uint16_t *dotx1,uint16_t *doty1,uint16_t dot_cnt,struct LINE_STRUCT *p);
void GUI_FcolorConDots2(uint16_t *dotx1,uint16_t *doty1,uint16_t dot_cnt,struct LINE_STRUCT *p);
void Write_quxian(uint16_t screen_id,uint16_t control_id,int16_t *dotx1,int16_t *doty1,uint16_t dot_cnt,struct LINE_STRUCT *p);
void Write_quxianC(uint16_t screen_id,uint16_t control_id,uint16_t color,int16_t *dotx1,int16_t *doty1,uint16_t dot_cnt,struct LINE_STRUCT *p); //ע��dot_cntΪʵ�ʸ������������0��ʼ
void Write_quxian_ENABLE(uint16_t X,uint16_t Y,uint16_t W,uint16_t H);
void Write_quxian1(uint16_t screen_id,uint16_t control_id,uint16_t X0,uint16_t X1,uint16_t Y0,uint16_t Y1);
void Clear_quxian(uint16_t screen_id,uint16_t control_id);
void tanchuang(uint16_t screen_id);
void biaoge_bottom(uint16_t screen_id,uint16_t control_id,uint16_t hangshu);
void Connect_return(void);
void NoConnect_return(void);

void banben_return(void)  ;
void shebeibanben_return(void) ;
void return_xishu(uint16_t k1,uint16_t b1,uint16_t k2,uint16_t b2)  ;
void system_return(void);
void Record_Delete(uint16_t screen_id, uint16_t control_id, uint16_t row);//删除表格中的数据

struct LINE_Float_STRUCT{

float QUXIAN_L_UP_X ;
float QUXIAN_L_UP_Y ;
float QUXIAN_R_DOWN_X ;
float QUXIAN_R_DOWN_Y ;

float  max_X;
float  max_Y;
float  min_X;
float  min_Y;
};

void Write_quxianC_float(uint16_t screen_id, uint16_t control_id, uint16_t color,uint16_t *dotx1, uint16_t *doty1, uint16_t dot_cnt,struct LINE_Float_STRUCT *p);

#endif /* BSP_HMI_DRIVER_HMI_DRIVER_H_ */
