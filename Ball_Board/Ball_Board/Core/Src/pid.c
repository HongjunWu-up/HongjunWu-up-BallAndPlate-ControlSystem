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
//位置式PID 

unsigned int k=50;
float PID_Pos_Control(PIDtype *pid,float realval,float target)
{
	     float pe, ie, de;  
       float out=0; 
       float adjust_out=0;
	
			//计算误差
      pid->err=target-realval;
	    //计算误差积分
	    pid->errsam+=pid->err;
		    if(pid->errsam>100000)
			{
				pid->errsam=100000;//积分限幅
			}
			if(pid->errsam<-100000)
			{
				pid->errsam=-100000;//积分限幅
			}
//	    if(pid->errsam* pid->Ki>1)
//			{
//				pid->errsam=1/ pid->Ki;//积分限幅
//			}
//			if(pid->errsam* pid->Ki<-1)
//			{
//				pid->errsam=-1/ pid->Ki;//积分限幅
//			}
			//此处考虑积分分离，误差较大时不加入积分。坐标误差大于10时，积分分离。
	    if(((realval- target) > 20.0)||(( target- realval) > 20.0))
			{
				pid->errsam = 0;
			}
			//计算误差微分
	    de=pid->err-pid->last_err;
	    pe = pid->err;  
      ie = pid->errsam;  
	    
	    pid->last_err=pid->err;
			
      //pid控制器	
      out= pe*(pid->Kp) + ie*(pid->Ki) + de*(pid->Kd);  
	
			return out;
}
/*1.不用积分分离会遇到什么问题
在普通PID控制中引入积分环节，主要是为了消除净差，提高控制精度。但在程序
启动，或大幅度增减目标值时，短时间内会造成系统有特别大的偏差，会造成PID
运算的积分积累，致使输出量特别大，很容易导致超调，甚至会引起系统较大的震荡。
2.积分分离的原理
当误差值比较大时，取消积分作用，以免由于积分作用使得系统稳定性降低，超
调量增大；当被控量接近目标值时（即误差较小时），引入积分控制，以消除净
差，提高控制精度。*/
//增量式PID
float PID_SPEED_Control(PIDtype *pid,float realval,float target)
{
	     float pe, ie, de;  
       float out=0; 
       float adjust_out=0;
	
			//计算误差
      pid->err=target-realval;
	    //计算误差积分
//	    pid->errsam+=pid->err;
//	    if(pid->errsam>k*target)pid->errsam=k*target;//积分限幅
//			//此处考虑积分分离，误差较大时不加入积分。
//			//计算误差微分
	    de=pid->err-pid->last_err;
	    pe = pid->err;  
      ie = pid->errsam;  
	    pid->last_err=pid->err;
			
      //pid控制器	
      out= pe*(pid->Kp) + ie*(pid->Ki) + de*(pid->Kd);  
			//输出限幅
//	    if(adjust_out>pid->maxval)adjust_out=pid->maxval;
//			if(adjust_out<pid->minval)adjust_out=pid->minval;
			
			return out;
}
float PID_R_MOVE(PIDtype *pid,float realval,float target)
{
	     float pe, ie, de;  
       float out=0; 
	
			//计算误差
      pid->err=target-realval;
	    //计算误差积分
	    pid->errsam+=pid->err;
		  if(pid->errsam* pid->Ki>1)
			{
				pid->errsam=1/ pid->Ki;//积分限幅
			}
			if(pid->errsam* pid->Ki<-1)
			{
				pid->errsam=-1/ pid->Ki;//积分限幅
			}
			//此处考虑积分分离，误差较大时不加入积分。坐标误差大于10时，积分分离。
	    if(((realval- target) > 10.0)||(( target- realval) > 10.0))
			{
				pid->errsam = 0;
			}
			//计算误差微分
	    de=pid->err-pid->last_err;
	    pe = pid->err;  
      ie = pid->errsam;  
			
	    pid->last_err=pid->err;
			
      //pid控制器	
      out= pe*(pid->Kp) + ie*(pid->Ki) + de*(pid->Kd);  
			//输出限幅
//	    if(out>pid->maxval)adjust_out=pid->maxval;
//			if(out<pid->minval)adjust_out=pid->minval;
			
			return out;
}
int PID_Add_Control(PIDtype *pid,float realval,float target)
{
       int out=0; 
			//计算误差
      pid->err=target-realval;
	
		  out= pid->Kp* (pid->err-pid->last_err)+ pid->Ki* pid->err
    	+ pid->Kd*(pid->err-2*pid->last_err+ pid->prev_err);  
	    pid->prev_err=pid->last_err;
	    pid->last_err=pid->err;
			
      //pid控制器	
      
			//输出限幅
	    if(out>pid->maxval)out=pid->maxval;
			if(out<pid->minval)out=pid->minval;
			
			return out;
}
int V_Xianfu_Pulse_Number1(int Pulse_Number)
{	
	  static int Amplitude = 700;    //===PWM满幅是1000 限制在900

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
	  static int Amplitude = 1439;    //===PWM满幅是1000 限制在900

    if(Pulse_Number < 0) 
			Pulse_Number = -Pulse_Number;
		
    if(0<=Pulse_Number && Pulse_Number<100) 
			Pulse_Number = 80;			

		if(Pulse_Number > Amplitude)  
			Pulse_Number = Amplitude;
		

		return Pulse_Number;
}
