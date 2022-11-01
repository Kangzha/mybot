#include "pid.h"

struct pid_uint pid_Task_Left;
struct pid_uint pid_Task_Right;

/****************************************************************************
*函数名称：PID_Init(void)
*函数功能：初始化PID结构体参数
****************************************************************************/

void PID_Init(void)
{

/***********************左轮速度pid****************************/
	pid_Task_Left.Kp = 1024 * 0.26;
 	pid_Task_Left.Ki = 1024 * 0;	
	pid_Task_Left.Kd = 1024 * 0.9; 
	pid_Task_Left.Ur = 1024 * 4000;
	pid_Task_Left.Adjust   = 0;
	pid_Task_Left.En       = 1;
	pid_Task_Left.speedSet = 0;
	pid_Task_Left.speedNow = 0;
	reset_Uk(&pid_Task_Left);		
/***********************右轮速度pid****************************/
	pid_Task_Right.Kp = 1024 * 0.26;
 	pid_Task_Right.Ki = 1024 * 0;	//不使用积分
	pid_Task_Right.Kd = 1024 * 0.9; 
	pid_Task_Right.Ur = 1024 * 4000;
	pid_Task_Right.Adjust   = 0;
	pid_Task_Right.En       = 1;
	pid_Task_Right.speedSet = 0;
	pid_Task_Right.speedNow = 0;
	reset_Uk(&pid_Task_Right);
}

/***********************************************************************************************
 函 数 名：void reset_Uk(PID_Uint *p)
 功    能：初始化U_kk,ekk,ekkk
 说    明：在初始化时调用，改变PID参数时有可能需要调用
 入口参数：PID单元的参数结构体 地址
************************************************************************************************/

void reset_Uk(struct pid_uint *p)
{
	p->U_kk=0;
	p->ekk=0;
	p->ekkk=0;
}

/***********************************************************************************************
 函 数 名：s32 PID_commen(int set,int jiance,PID_Uint *p)
 功    能：PID计算函数
 说    明：求任意单个PID的控制量
 入口参数：期望值，实测值，PID单元结构体
 返 回 值：PID控制量
************************************************************************************************/

s32 PID_common(int set,int jiance,struct pid_uint *p)
{
	int ek=0,U_k=0;

	ek=jiance - set;                                                               
	
	U_k=p->U_kk + p->Kp*(ek - p->ekk) + p->Ki*ek + p->Kd*(ek - 2*p->ekk + p->ekkk);
	
	p->U_kk=U_k;
    p->ekkk=p->ekk;
	p->ekk=ek;
	
	if(U_k>(p->Ur))		                                    
		U_k=p->Ur;
	if(U_k<-(p->Ur))
		U_k=-(p->Ur);
	
	return U_k>>10; //之前乘1024，右移10位除1024
}

/***********************************************************************************
** 函数名称 ：void Pid_Which(struct pid_uint *pl, struct pid_uint *pr)
** 函数功能 ：pid选择函数	      
***********************************************************************************/

void Pid_Which(struct pid_uint *pl, struct pid_uint *pr)
{
	/**********************左轮速度pid*************************/
	if(pl->En == 1)
	{									
		pl->Adjust = -PID_common(pl->speedSet, pl->speedNow, pl);		
	}	
	else
	{
		pl->Adjust = 0;
		reset_Uk(pl);
		pl->En = 2; 
	}
	/***********************右轮速度pid*************************/
	if(pr->En == 1)
	{
		pr->Adjust = -PID_common(pr->speedSet, pr->speedNow, pr);		
	}	
	else
	{
		pr->Adjust = 0;
		reset_Uk(pr);
		pr->En = 2; 
	}
}

/*******************************************************************************
 * 函数名：Pid_Ctrl(int *leftMotor,int  *rightMotor)
 * 描述  ：Pid控制
 *******************************************************************************/

void Pid_Ctrl(int *leftMotor,int  *rightMotor)
{
	Pid_Which(&pid_Task_Left, &pid_Task_Right); 
	*leftMotor  += pid_Task_Left.Adjust;
	*rightMotor += pid_Task_Right.Adjust;
}