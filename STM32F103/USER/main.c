#include "stm32f10x_flash.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "inv_mpu.h"
#include "mpu9250.h"
#include "LinuxUsart.h"
#include "timer.h"
#include "pwm.h"
#include "motor.h"
#include "storage_manager.h"
#include "pid.h"
//#include "myexti.h"
#define M  1000
  //速度预设值，由上位机发出 
  int leftSpeedSet=0;  
  int rightSpeedSet=0;
  int yaw_=1;
	//速度测量值  mm / s
	int leftSpeedNow=0;
	int rightSpeedNow=0;
	unsigned char testRece3=0x01;
	float pitch,roll,yaw; 		//欧拉角
	short sAcc[3],sGyro[3],sMag[3];//寄存器原始数据
	short temp;					//温度
	
	int left_dir=0,right_dir=0;   //轮子方向
	int left_speed_count=0;
	int right_speed_count=0;  //脉冲数
	
	unsigned char mpudataBuf[125];
	
	
int main(void)
{ 
	u8 t=0;
	
 	int flag=0;  //判断校准数据是否已经保存					
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init();      //初始化延时函数
	usart_init(115200);		//初始化串口波特率为115200
	Motor_Init(7199,0);             //初始化PWM 10KHZ，驱动电机
	PID_Init();
  TIM5_Cap_Init(10000,72-1); //10ms 100HZ
	TIM5_Cap_GPIO_Init();
	//MPU9250_Init();
	while(mpu_dmp_init()){;}
	for(int i=0;i<100;++i)
	{
		mpu_dmp_get_data(&pitch,&roll,&yaw);
	}
  
	yaw_ = yaw;
  while(1) 
	{
		if(mpu_dmp_get_data_(&pitch,&roll,&yaw,yaw_)==0)
		{
			t++;
			
		}
		
		if(t==15)
		{									
			UsartSendData(yaw*M,leftSpeedNow,rightSpeedNow,yaw_*1000);
		  t=0;
		}
  }
}
//串口中断
void USART1_IRQHandler()
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
 	 {
		 USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		 //从ROS接收到的数据，存放到下面三个变量中
		 usartReceiveOneData(&leftSpeedSet,&rightSpeedSet,&testRece3);
	 }
}
//测速中断
void TIM5_IRQHandler(void)                            
{
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);  //定时器10ms中断捕获
		GetSpeedInfo(&leftSpeedNow,left_speed_count,left_dir);
		GetSpeedInfo(&rightSpeedNow,right_speed_count,right_dir);
		left_speed_count=0;
		right_speed_count=0;
		pid_Task_Left.speedSet  = leftSpeedSet;
		pid_Task_Right.speedSet = rightSpeedSet;
		pid_Task_Left.speedNow  = leftSpeedNow;
		pid_Task_Right.speedNow = rightSpeedNow;
		Pid_Ctrl(&motorLeft,&motorRight);
		Set_Pwm(motorLeft,motorRight);
	}
	else if(TIM_GetITStatus(TIM5, TIM_IT_CC3) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);  //通道3捕获
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_4)==1)
		{
			left_dir=1; //向后转  装电机时方向相反
			left_speed_count++;
		}
		else
		{
			left_dir=0; //向前转
			left_speed_count++;
		}
	}
	else if(TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);  //通道4捕获
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15)==1)
		{
			right_dir=0; //向后转
			right_speed_count++;
		}
		else
		{
			right_dir=1; //向前转
			right_speed_count++;
		}
	}
}

