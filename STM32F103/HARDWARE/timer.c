#include "timer.h"

//arr：自动重装值。
//psc：时钟预分频数
void TIM5_Cap_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM5_ICInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;   //TIM5_CH3 //TIM5_CH4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;         //复用推挽输出
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr;                	 
	TIM_TimeBaseStructure.TIM_Prescaler =psc;              
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 

	TIM5_ICInitStructure.TIM_Channel=TIM_Channel_3;
	TIM5_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM5_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM5_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM5_ICInitStructure.TIM_ICFilter=0x00;
	TIM_ICInit(TIM5, &TIM5_ICInitStructure);
	
	TIM5_ICInitStructure.TIM_Channel=TIM_Channel_4;
	TIM5_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM5_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM5_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM5_ICInitStructure.TIM_ICFilter=0x00;
	TIM_ICInit(TIM5, &TIM5_ICInitStructure);
	
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;            //TIM5中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM5, TIM_IT_Update|TIM_IT_CC3|TIM_IT_CC4, ENABLE);
	TIM_Cmd(TIM5,ENABLE);
}

void TIM5_Cap_GPIO_Init()
{
	
	GPIO_InitTypeDef GPIO_InitStructure1;
	GPIO_InitStructure1.GPIO_Pin = GPIO_Pin_4;   
	GPIO_InitStructure1.GPIO_Mode = GPIO_Mode_IPD;         
	GPIO_InitStructure1.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure1);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;         
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);			
}
void GetSpeedInfo(int *speed,int count,int dir)
{
	if(dir)
		*speed = count*100*3.1416*67.3/330;   // mm/s
	else
		*speed = -count*100*3.1416*67.3/330;
}