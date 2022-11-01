#include "LinuxUsart.h"
#include "usart.h"

//数据接收暂存区
unsigned char  receiveBuff[16] = {0};         
//通信协议常量
const unsigned char header[2]  = {0x55, 0xaa};
const unsigned char ender[2]   = {0x0d, 0x0a};

//*****************发送数据（左轮速、右轮速、角度）共用体（-32767 - +32768）
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
//******************左右轮速控制速度共用体
union receiveData
{
	short d;
	unsigned char data[2];
}leftVelSet,rightVelSet;


//************************************获取上位机数据
int usartReceiveOneData(int *p_leftSpeedSet,int *p_rightSpeedSet,unsigned char *p_crtlFlag)
{
	unsigned char USART_Receiver              = 0;          //接收数据
	static unsigned char checkSum             = 0;
	static unsigned char USARTBufferIndex     = 0;
	static short j=0,k=0;
	static unsigned char USARTReceiverFront   = 0;
	static unsigned char Start_Flag           = START;      //一帧数据传送开始标志位
	static short dataLength                   = 0;

	USART_Receiver = USART_ReceiveData(USART1);   //可能根据实际情况修改
	//接收消息头
	if(Start_Flag == START)
	{
		if(USART_Receiver == 0xaa)                             //buf[1]
		{  
			if(USARTReceiverFront == 0x55)        //数据头两位 //buf[0]
			{
				Start_Flag = !START;              //收到数据头，开始接收数据
				//printf("header ok\n");
				receiveBuff[0]=header[0];         //buf[0]
				receiveBuff[1]=header[1];         //buf[1]
				USARTBufferIndex = 0;             //缓冲区初始化
				checkSum = 0x00;				  //校验和初始化
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
			case 0://接收左右轮速度数据的长度
				receiveBuff[2] = USART_Receiver;
				dataLength     = receiveBuff[2];            //buf[2]
				USARTBufferIndex++;
				break;
			case 1://接收所有数据，并赋值处理 
				receiveBuff[j + 3] = USART_Receiver;        //buf[3] - buf[7]					
				j++;
				if(j >= dataLength)                         
				{
					j = 0;
					USARTBufferIndex++;
				}
				break;
			case 2://接收校验值信息
				receiveBuff[3 + dataLength] = USART_Receiver;
				checkSum = getCrc8(receiveBuff, 3 + dataLength);
				  // 检查信息校验值
				if (checkSum != receiveBuff[3 + dataLength]) //buf[8]
				{
					printf("Received data check sum error!");
					return 0;
				}
				USARTBufferIndex++;
				break;
				
			case 3://接收信息尾
				if(k==0)
				{
					//数据0d     buf[9]  无需判断
					k++;
				}
				else if (k==1)
				{
					//数据0a     buf[10] 无需判断

					//进行速度赋值操作					
					 for(k = 0; k < 2; k++)
					{
						leftVelSet.data[k]  = receiveBuff[k + 3]; //buf[3]  buf[4]
						rightVelSet.data[k] = receiveBuff[k + 5]; //buf[5]  buf[6]
					}				
					
					//速度赋值操作
					*p_leftSpeedSet  = (int)leftVelSet.d;
					*p_rightSpeedSet = (int)rightVelSet.d;
					
					//ctrlFlag
					*p_crtlFlag = receiveBuff[7];                //buf[7]
					
					//-----------------------------------------------------------------
					//完成一个数据包的接收，相关变量清零，等待下一字节数据
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


//************************************发送数据给上位机（test）

void UsartSendData(int yaw, int left_speed,int right_speed,int flag)
{ 
  // 协议数据缓存数组   左右轮速度、yaw\roll\pitch角度、xyz三个方向加速度、预留控制位
	unsigned char buf[22]={0};
	int i, length = 0;
	
	//leftVelNow.d  = leftVel;
	//rightVelNow.d = rightVel;
	Yaw.d         = yaw;
	Left_Speed.d        = left_speed;
	Right_Speed.d        = right_speed;
	Flag.d        = flag;

	//设置消息头
	for(i = 0; i < 2; i++)
		buf[i] = header[i];                      // buf[0] buf[1]
	//设置数据主体
	length = 16;
	buf[2] = length; 	//buf[2]
	for(i=0;i<4;i++)  //buf[3] ~ buf[18
	{
		buf[i+3] = Yaw.data[i];
		buf[i+7] = Left_Speed.data[i];
		buf[i+11]= Right_Speed.data[i];
		buf[i+15] = Flag.data[i];
	}
	
	// 设置校验值、消息尾
	buf[3 + length] = getCrc8(buf, 3 + length);  // buf[19
	buf[3 + length + 1] = ender[0];              // buf[20
	buf[3 + length + 2] = ender[1];              // buf[21
	
	//发送字符串数据
	USART_Send_String(buf,sizeof(buf));
}


//*************************发送指定长度的字符数组，被usartSendData调用
void USART_Send_String(u8 *p,u16 sendSize)
{ 
	static int length =0;
	while(length<sendSize)
	{   
		while( !(USART1->SR&(0x01<<7)) );//发送完成位：TXE
		USART1->DR=*p;                   
		p++;
		length++;
	}
	length =0;
}

//***********************计算八位循环冗余校验，被usartSendData和usartReceiveOneData函数调用
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

