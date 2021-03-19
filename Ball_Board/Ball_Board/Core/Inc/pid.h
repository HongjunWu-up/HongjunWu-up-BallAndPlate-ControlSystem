//#ifndef __PID_H
//#define __PID_H
//   
//typedef struct
//{
//  double SetPoint;//�趨Ŀ��ֵ
//  double Proportion;//��������
//	double Integral;//���ֳ���
//	double Derivative;//΢�ֳ���
//	double LastError;//Error[-1]
//	double PrevError;//Error[-2]
//	double SumError;//������
//}PID; 


//double PIDCalc(PID *P,double NextPoint);	 
//#endif
#ifndef __PID_H
#define __PID_H



typedef struct 
{
	 unsigned short maxval;
	 unsigned short minval;
	 float err;
	 float last_err;
	 float prev_err;
	 float Kp,Ki,Kd;
	 float errsam;
}PIDtype;

void PID_Init(PIDtype *pid,unsigned short max,unsigned short min,float kp,float ki,float kd);
void PID_SET(PIDtype *pid,unsigned short max,unsigned short min,float kp,float ki,float kd);
float  PID_SPEED_Control(PIDtype *pid,float realval,float target);//λ��ʽPID
float  PID_Pos_Control(PIDtype *pid,float realval,float target);//λ��ʽPID
float PID_R_MOVE(PIDtype *pid,float realval,float target);
int PID_Add_Control(PIDtype *pid,float realval,float target);//����ʽpid
int V_Xianfu_Pulse_Number1(int Pulse_Number);
int I_Xianfu_Pulse_Number1(int Pulse_Number);
#endif
