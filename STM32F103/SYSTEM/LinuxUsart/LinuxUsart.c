#include "LinuxUsart.h"
#include "usart.h"

//���ݽ����ݴ���
unsigned char  receiveBuff[16] = {0};         
//ͨ��Э�鳣��
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};

//*****************�������ݣ������١������١��Ƕȣ������壨-32767 - +32768��
//union sendData
//{
//	short d;
//	unsigned char data[2];
//}leftVelNow,rightVelNow,angleNow,Yaw0,Roll0,Pitch0,X,Y,Z;
union sendData
{
	short d;
	unsigned char data[2];
}Yaw0,Roll0,Pitch0,Accx,Accy,Accz,Gyrox,Gyroy,Gyroz,Magx,Magy,Magz,sFlag;
union SendData
{
  int d;
	unsigned char data[4];
}Yaw,Left_Speed,Right_Speed,Flag;
//******************�������ٿ����ٶȹ�����
union receiveData
{
	short d;
	unsigned char data[2];
}leftVelSet,rightVelSet;


//************************************��ȡ��λ������
int usartReceiveOneData(int *p_leftSpeedSet,int *p_rightSpeedSet,unsigned char *p_crtlFlag)
{
	unsigned char USART_Receiver              = 0;          //��������
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //һ֡���ݴ��Ϳ�ʼ��־λ
	static short dataLength                   = 0;

	USART_Receiver = USART_ReceiveData(USART1);   //���ܸ���ʵ������޸�
	//������Ϣͷ
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)        //����ͷ��λ //buf[0]
			{
				Start_Flag = !START;              //�յ�����ͷ����ʼ��������
				//printf("header ok\n");
				receiveBuff[0]=header[0];         //buf[0]
				receiveBuff[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //��������ʼ��
				checkSum = 0x00;				  //У��ͳ�ʼ��
			}
		}
		else 
		{
			USARTReceiverFront = USART_Receiver;  
		}
	}
	else
    { 
		switch(USARTBufferIndex)
		{
			case 0://�����������ٶ����ݵĳ���
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://�����������ݣ�����ֵ���� 
				receiveBuff[j + 3] = USART_Receiver;        //buf[3] - buf[7]					
				j++;
				if(j >= dataLength)                         
				{
					j = 0;
					USARTBufferIndex++;
				}
				break;
			case 2://����У��ֵ��Ϣ
				receiveBuff[3 + dataLength] = USART_Receiver;
				checkSum = getCrc8(receiveBuff, 3 + dataLength);
				  // �����ϢУ��ֵ
				if (checkSum != receiveBuff[3 + dataLength]) //buf[8]
				{
					printf("Received data check sum error!");
					return 0;
				}
				USARTBufferIndex++;
				break;
				
			case 3://������Ϣβ
				if(k==0)
				{
					//����0d     buf[9]  �����ж�
					k++;
				}
				else if (k==1)
				{
					//����0a     buf[10] �����ж�

					//�����ٶȸ�ֵ����					
					 for(k = 0; k < 2; k++)
					{
						leftVelSet.data[k]  = receiveBuff[k + 3]; //buf[3]  buf[4]
						rightVelSet.data[k] = receiveBuff[k + 5]; //buf[5]  buf[6]
					}				
					
					//�ٶȸ�ֵ����
					*p_leftSpeedSet  = (int)leftVelSet.d;
					*p_rightSpeedSet = (int)rightVelSet.d;
					
					//ctrlFlag
					*p_crtlFlag = receiveBuff[7];                //buf[7]
					
					//-----------------------------------------------------------------
					//���һ�����ݰ��Ľ��գ���ر������㣬�ȴ���һ�ֽ�����
					USARTBufferIndex   = 0;
					USARTReceiverFront = 0;
					Start_Flag         = START;
					checkSum           = 0;
					dataLength         = 0;
					j = 0;
					k = 0;
					//-----------------------------------------------------------------					
				}
				break;
			 default:break;
		}		
	}
	return 0;
}


//************************************�������ݸ���λ����test��

void UsartSendData(int yaw, int left_speed,int right_speed,int flag)
{ 
  // Э�����ݻ�������   �������ٶȡ�yaw\roll\pitch�Ƕȡ�xyz����������ٶȡ�Ԥ������λ
	unsigned char buf[22]={0};
	int i, length = 0;
	
	//leftVelNow.d  = leftVel;
	//rightVelNow.d = rightVel;
	Yaw.d         = yaw;
	Left_Speed.d        = left_speed;
	Right_Speed.d        = right_speed;
	Flag.d        = flag;

	//������Ϣͷ
	for(i = 0; i < 2; i++)
		buf[i] = header[i];                      // buf[0] buf[1]
	//������������
	length = 16;
	buf[2] = length; 	//buf[2]
	for(i=0;i<4;i++)  //buf[3] ~ buf[18
	{
		buf[i+3] = Yaw.data[i];
		buf[i+7] = Left_Speed.data[i];
		buf[i+11]= Right_Speed.data[i];
		buf[i+15] = Flag.data[i];
	}
	
	// ����У��ֵ����Ϣβ
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[19
	buf[3 + length + 1] = ender[0];              // buf[20
	buf[3 + length + 2] = ender[1];              // buf[21
	
	//�����ַ�������
	USART_Send_String(buf,sizeof(buf));
}


//*************************����ָ�����ȵ��ַ����飬��usartSendData����
void USART_Send_String(u8 *p,u16 sendSize)
{ 
	static int length =0;
	while(length<sendSize)
	{   
		while( !(USART1->SR&(0x01<<7)) );//�������λ��TXE
		USART1->DR=*p;                   
		p++;
		length++;
	}
	length =0;
}

//***********************�����λѭ������У�飬��usartSendData��usartReceiveOneData��������
unsigned char getCrc8(unsigned char *ptr, unsigned short len)
{
	unsigned char crc;
	unsigned char i;
	crc = 0;
	while(len--)
	{
		crc ^= *ptr++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
                crc=(crc>>1)^0x8C;
			else 
                crc >>= 1;
		}
	}
	return crc;
}

