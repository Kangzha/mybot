#ifndef __LINUXUSART__
#define __LINUXUSART__
#include <sys.h>	

#define START   0X11

//��linux���ղ��������ݵ�������ַ��
extern int usartReceiveOneData(int *p_leftSpeedSet,int *p_rightSpeedSet,unsigned char *p_crtlFlag);   
//��װ���ݣ�����USART1_Send_String�����ݷ��͸�linux
extern void usartSendData(short leftVel, short rightVel,short angle,unsigned char ctrlFlag); 
extern void UsartSendData(int yaw, int left_speed,int right_speed,int flag);
 
//����ָ���ַ�����ĺ���
void USART_Send_String(unsigned char *p,unsigned short sendSize);     
//�����λѭ������У�飬�õ�У��ֵ��һ���̶�����֤���ݵ���ȷ��
unsigned char getCrc8(unsigned char *ptr, unsigned short len); 

#endif