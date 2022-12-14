#include "pwm.h"
//   *************************TB6612电机驱动器初始化 
void Motor_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能PC端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;	//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOC, &GPIO_InitStructure);				  //根据设定参数初始化GPIOC
	
	GPIO_InitTypeDef GPIO_InitStructure1;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能PC端口时钟
	GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;	//端口配置
	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_Out_PP;      //推挽输出
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;     //50M
	GPIO_Init(GPIOB, &GPIO_InitStructure1);				  //根据设定参数初始化GPIOC
	
	PWM_Init(arr,psc);
}

//  ************************控制两路电机的PWM初始化
void PWM_Init(u16 arr,u16 psc)
{		 		
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);    // 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE);  //使能GPIO外设时钟使能
	//设置该引脚为复用输出功能,输出TIM1 CH1 CH4的PWM脉冲波形
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;   //TIM3_CH3 //TIM3_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = arr;                //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 
	TIM_TimeBaseStructure.TIM_Prescaler =psc;              //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;           //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);        //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;      //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);               //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);               //根据TIM_OCInitStruct中指定的参数初始化外设TIMx

	TIM_CtrlPWMOutputs(TIM3,ENABLE);	                  //MOE 主输出使能	

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);     //CH3预装载使能	 
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);     //CH4预装载使能	 

	TIM_ARRPreloadConfig(TIM3, ENABLE);                   //使能TIMx在ARR上的预装载寄存器

	TIM_Cmd(TIM3, ENABLE);                                //使能TIM1
 
}











