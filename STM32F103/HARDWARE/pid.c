#include "pid.h"

struct pid_uint pid_Task_Left;
struct pid_uint pid_Task_Right;

/****************************************************************************
*�������ƣ�PID_Init(void)
*�������ܣ���ʼ��PID�ṹ�����
****************************************************************************/

void PID_Init(void)
{

/***********************�����ٶ�pid****************************/
	pid_Task_Left.Kp = 1024 * 0.26;
 	pid_Task_Left.Ki = 1024 * 0;	
	pid_Task_Left.Kd = 1024 * 0.9; 
	pid_Task_Left.Ur = 1024 * 4000;
	pid_Task_Left.Adjust   = 0;
	pid_Task_Left.En       = 1;
	pid_Task_Left.speedSet = 0;
	pid_Task_Left.speedNow = 0;
	reset_Uk(&pid_Task_Left);		
/***********************�����ٶ�pid****************************/
	pid_Task_Right.Kp = 1024 * 0.26;
 	pid_Task_Right.Ki = 1024 * 0;	//��ʹ�û���
	pid_Task_Right.Kd = 1024 * 0.9; 
	pid_Task_Right.Ur = 1024 * 4000;
	pid_Task_Right.Adjust   = 0;
	pid_Task_Right.En       = 1;
	pid_Task_Right.speedSet = 0;
	pid_Task_Right.speedNow = 0;
	reset_Uk(&pid_Task_Right);
}

/***********************************************************************************************
 �� �� ����void reset_Uk(PID_Uint *p)
 ��    �ܣ���ʼ��U_kk,ekk,ekkk
 ˵    �����ڳ�ʼ��ʱ���ã��ı�PID����ʱ�п�����Ҫ����
 ��ڲ�����PID��Ԫ�Ĳ����ṹ�� ��ַ
************************************************************************************************/

void reset_Uk(struct pid_uint *p)
{
	p->U_kk=0;
	p->ekk=0;
	p->ekkk=0;
}

/***********************************************************************************************
 �� �� ����s32 PID_commen(int set,int jiance,PID_Uint *p)
 ��    �ܣ�PID���㺯��
 ˵    ���������ⵥ��PID�Ŀ�����
 ��ڲ���������ֵ��ʵ��ֵ��PID��Ԫ�ṹ��
 �� �� ֵ��PID������
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
	
	return U_k>>10; //֮ǰ��1024������10λ��1024
}

/***********************************************************************************
** �������� ��void Pid_Which(struct pid_uint *pl, struct pid_uint *pr)
** �������� ��pidѡ����	      
***********************************************************************************/

void Pid_Which(struct pid_uint *pl, struct pid_uint *pr)
{
	/**********************�����ٶ�pid*************************/
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
	/***********************�����ٶ�pid*************************/
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
 * ��������Pid_Ctrl(int *leftMotor,int  *rightMotor)
 * ����  ��Pid����
 *******************************************************************************/

void Pid_Ctrl(int *leftMotor,int  *rightMotor)
{
	Pid_Which(&pid_Task_Left, &pid_Task_Right); 
	*leftMotor  += pid_Task_Left.Adjust;
	*rightMotor += pid_Task_Right.Adjust;
}