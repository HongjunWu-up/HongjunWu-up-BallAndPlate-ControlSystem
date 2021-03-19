#include "pid.h"
#include "stdlib.h"


void PID_Init(PIDtype *pid,unsigned short max,unsigned short min,float kp,float ki,float kd)
{
	 pid->prev_err=0; 
	 pid->last_err=0;
	 pid->errsam=0;
	 pid->Kp=kp;
	 pid->Ki=ki;
	 pid->Kd=kd;
	 pid->maxval=max;
	 pid->minval=min;

	 
}
void PID_SET(PIDtype *pid,unsigned short max,unsigned short min,float kp,float ki,float kd)
{
	 pid->Kp=kp;
	 pid->Ki=ki;
	 pid->Kd=kd;
	 pid->maxval=max;
	 pid->minval=min;
	 
}
//λ��ʽPID 

unsigned int k=50;
float PID_Pos_Control(PIDtype *pid,float realval,float target)
{
	     float pe, ie, de;  
       float out=0; 
       float adjust_out=0;
	
			//�������
      pid->err=target-realval;
	    //����������
	    pid->errsam+=pid->err;
		    if(pid->errsam>100000)
			{
				pid->errsam=100000;//�����޷�
			}
			if(pid->errsam<-100000)
			{
				pid->errsam=-100000;//�����޷�
			}
//	    if(pid->errsam* pid->Ki>1)
//			{
//				pid->errsam=1/ pid->Ki;//�����޷�
//			}
//			if(pid->errsam* pid->Ki<-1)
//			{
//				pid->errsam=-1/ pid->Ki;//�����޷�
//			}
			//�˴����ǻ��ַ��룬���ϴ�ʱ��������֡�����������10ʱ�����ַ��롣
	    if(((realval- target) > 20.0)||(( target- realval) > 20.0))
			{
				pid->errsam = 0;
			}
			//�������΢��
	    de=pid->err-pid->last_err;
	    pe = pid->err;  
      ie = pid->errsam;  
	    
	    pid->last_err=pid->err;
			
      //pid������	
      out= pe*(pid->Kp) + ie*(pid->Ki) + de*(pid->Kd);  
	
			return out;
}
/*1.���û��ַ��������ʲô����
����ͨPID������������ֻ��ڣ���Ҫ��Ϊ�����������߿��ƾ��ȡ����ڳ���
����������������Ŀ��ֵʱ����ʱ���ڻ����ϵͳ���ر���ƫ������PID
����Ļ��ֻ��ۣ���ʹ������ر�󣬺����׵��³���������������ϵͳ�ϴ���𵴡�
2.���ַ����ԭ��
�����ֵ�Ƚϴ�ʱ��ȡ���������ã��������ڻ�������ʹ��ϵͳ�ȶ��Խ��ͣ���
�������󣻵��������ӽ�Ŀ��ֵʱ��������Сʱ����������ֿ��ƣ���������
���߿��ƾ��ȡ�*/
//����ʽPID
float PID_SPEED_Control(PIDtype *pid,float realval,float target)
{
	     float pe, ie, de;  
       float out=0; 
       float adjust_out=0;
	
			//�������
      pid->err=target-realval;
	    //����������
//	    pid->errsam+=pid->err;
//	    if(pid->errsam>k*target)pid->errsam=k*target;//�����޷�
//			//�˴����ǻ��ַ��룬���ϴ�ʱ��������֡�
//			//�������΢��
	    de=pid->err-pid->last_err;
	    pe = pid->err;  
      ie = pid->errsam;  
	    pid->last_err=pid->err;
			
      //pid������	
      out= pe*(pid->Kp) + ie*(pid->Ki) + de*(pid->Kd);  
			//����޷�
//	    if(adjust_out>pid->maxval)adjust_out=pid->maxval;
//			if(adjust_out<pid->minval)adjust_out=pid->minval;
			
			return out;
}
float PID_R_MOVE(PIDtype *pid,float realval,float target)
{
	     float pe, ie, de;  
       float out=0; 
	
			//�������
      pid->err=target-realval;
	    //����������
	    pid->errsam+=pid->err;
		  if(pid->errsam* pid->Ki>1)
			{
				pid->errsam=1/ pid->Ki;//�����޷�
			}
			if(pid->errsam* pid->Ki<-1)
			{
				pid->errsam=-1/ pid->Ki;//�����޷�
			}
			//�˴����ǻ��ַ��룬���ϴ�ʱ��������֡�����������10ʱ�����ַ��롣
	    if(((realval- target) > 10.0)||(( target- realval) > 10.0))
			{
				pid->errsam = 0;
			}
			//�������΢��
	    de=pid->err-pid->last_err;
	    pe = pid->err;  
      ie = pid->errsam;  
			
	    pid->last_err=pid->err;
			
      //pid������	
      out= pe*(pid->Kp) + ie*(pid->Ki) + de*(pid->Kd);  
			//����޷�
//	    if(out>pid->maxval)adjust_out=pid->maxval;
//			if(out<pid->minval)adjust_out=pid->minval;
			
			return out;
}
int PID_Add_Control(PIDtype *pid,float realval,float target)
{
       int out=0; 
			//�������
      pid->err=target-realval;
	
		  out= pid->Kp* (pid->err-pid->last_err)+ pid->Ki* pid->err
    	+ pid->Kd*(pid->err-2*pid->last_err+ pid->prev_err);  
	    pid->prev_err=pid->last_err;
	    pid->last_err=pid->err;
			
      //pid������	
      
			//����޷�
	    if(out>pid->maxval)out=pid->maxval;
			if(out<pid->minval)out=pid->minval;
			
			return out;
}
int V_Xianfu_Pulse_Number1(int Pulse_Number)
{	
	  static int Amplitude = 700;    //===PWM������1000 ������900

    if(Pulse_Number < 0) 
			Pulse_Number = -Pulse_Number;
		
    if(0<Pulse_Number && Pulse_Number<100) 
			Pulse_Number = 100;			

		if(Pulse_Number > Amplitude)  
			Pulse_Number = Amplitude;
		

		return Pulse_Number;
}
int I_Xianfu_Pulse_Number1(int Pulse_Number)
{	
	  static int Amplitude = 1439;    //===PWM������1000 ������900

    if(Pulse_Number < 0) 
			Pulse_Number = -Pulse_Number;
		
    if(0<=Pulse_Number && Pulse_Number<100) 
			Pulse_Number = 80;			

		if(Pulse_Number > Amplitude)  
			Pulse_Number = Amplitude;
		

		return Pulse_Number;
}
