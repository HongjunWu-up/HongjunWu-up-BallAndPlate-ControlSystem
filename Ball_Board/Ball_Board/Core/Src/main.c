/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "oled.h" 
#include "pid.h"
#include "DataScope_DP.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
	PIDtype X_POS_pid;//定义X轴位置环
	PIDtype Y_POS_pid;//定义Y轴位置环
	PIDtype X_SPEED_pid;//定义X轴速度环
	PIDtype Y_SPEED_pid;//定义Y轴速度环
	PIDtype R_MOVE_pid;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*模式定义*/
#define ceter_mode  1
#define square_mode 2
#define cycle_mode 3
#define light_mode 4
#define tangular_mode 5	
#define Up_mode 6
#define Down_mode 7
#define Left_mode 8
#define Right_mode 9
#define Control_mode 9

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//初始化X轴、Y轴位置外环和速度内环的PID参数
//float X_POS_Kp = 0.06,X_POS_Ki = 0.001,X_POS_Kd = 0.1,X_POS_Max = 140, X_POS_Min = -140;
//float Y_POS_Kp = 0.06,Y_POS_Ki = 0.001,Y_POS_Kd = 0.1,Y_POS_Max = 140, Y_POS_Min = -140;
//float X_SPENED_Kp = 10,X_SPENED_Ki = 0,X_SPENED_Kd = 5,X_SPENED_Max = 500, X_SPENED__Min = -500;
//float Y_SPENED_Kp = 10,Y_SPENED_Ki = 0,Y_SPENED_Kd = 5,Y_SPENED_Max = 500, Y_SPENED__Min = -500;
float X_POS_Kp = 0.065,X_POS_Ki = 0,X_POS_Kd = 0,X_POS_Max = 140, X_POS_Min = -140;
float Y_POS_Kp = 0.065,Y_POS_Ki = 0,Y_POS_Kd = 0,Y_POS_Max = 140, Y_POS_Min = -140;
float X_SPENED_Kp = 6,X_SPENED_Ki = 0,X_SPENED_Kd = 2,X_SPENED_Max = 500, X_SPENED__Min = -500;
float Y_SPENED_Kp = 6,Y_SPENED_Ki = 0,Y_SPENED_Kd = 2,Y_SPENED_Max = 500, Y_SPENED__Min = -500;
float R_MOVE_Kp = 0.1,R_MOVE_Ki = 0,R_MOVE_Kd = 0,R_MOVE_Max = 150, R_MOVE__Min = 150;
//图片设置240x240
float X_Center = 150,Y_Center = 150;//定义中心点坐标
double X_Pos = 150,Y_Pos = 150;//实际坐标参数定义
float Last_X_Pos = 150,Last_Y_Pos = 150;
double Light_X_Pos=150,Light_Y_Pos = 150;//激光坐标
float X_Speed = 0, Y_Speed = 0;//标配是实际的速度
float X_Target =150,Y_Target = 150;//目标点参数定义
float X_Speed_Set=0.0,Y_Speed_Set=0.0;//位置外环得到期望速度值
float X_PWM = 0,Y_PWM = 0;//速度内环得到输出的PWM
float out1=0,out2=0;
//通信数组定义（包括是否有球/球的坐标和速度）
float X_mid = 850.0, Y_mid = 830.0;//定义PWM输出处置，保持平板处于水平位置。
uint8_t hello[] = "USART1 is ready...\n";//串口发送接收缓冲区
uint8_t recv_buf[13] = {0};
uint8_t RxByte;
uint8_t RxBuff[255];
uint8_t RxBuff_hc[255];
uint16_t Rx_Count;
uint16_t Rx_Count_hc;
uint16_t test_angle=45;
uint8_t ball=0,light=0;//判断小球是否存在，激光是否存在
long int data_count=0;
unsigned char Mode=0;//模式选择
//模式变量设置
unsigned short Mode_flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void OLED_show(void);
void Servo1_Control(uint16_t angle);
void Servo2_Control(uint16_t angle);
void X_Servo_Control(float pwm);
void Y_Servo_Control(float pwm);
void dealcommand(void);
void openMVDataProess(uint8_t RxByte);
void HC05DataProess(uint8_t RxByte);
void Data_Debug(void);
void Change_mode(unsigned short mode );

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	unsigned i;
	unsigned flag=0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	PID_Init(&X_POS_pid,X_POS_Max,X_POS_Min,X_POS_Kp,X_POS_Ki,X_POS_Kd);//X轴、Y轴位置外环初始化
	PID_Init(&Y_POS_pid,Y_POS_Max,Y_POS_Min,Y_POS_Kp,Y_POS_Ki,Y_POS_Kd);
	PID_Init(&X_SPEED_pid,X_SPENED_Max,X_SPENED__Min,X_SPENED_Kp,X_SPENED_Ki,X_SPENED_Kd);//X轴、Y轴速度内环初始化	
	PID_Init(&Y_SPEED_pid,Y_SPENED_Max,Y_SPENED__Min,Y_SPENED_Kp,Y_SPENED_Ki,Y_SPENED_Kd);
	PID_Init(&R_MOVE_pid,R_MOVE_Max,R_MOVE__Min,R_MOVE_Kp,R_MOVE_Ki,R_MOVE_Kd);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	/*使能定时器1中断*/
	HAL_TIM_Base_Start_IT(&htim2);
//	Servo1_Control(0);
//	Servo2_Control(90);
  //使能串口中断接收
//  HAL_UART_Receive_IT(&huart3, (uint8_t*)recv_buf, 13);
	HAL_UART_Receive_IT(&huart3,&RxByte,1);
	HAL_UART_Receive_IT(&huart1,&RxByte,1);
  //发送提示信息
//  HAL_UART_Transmit_IT(&huart3, (uint8_t*)hello, sizeof(hello));
//  HAL_UART_Transmit_IT(&huart1, (uint8_t*)hello, sizeof(hello));
	X_Servo_Control(X_mid);
	Y_Servo_Control(Y_mid);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
// 板面运动测试代码
//		for (i=0;i<90;i++)
//		{
//		Servo1_Control(i);	
//		Servo2_Control(90-i);	
//		HAL_Delay(10);
//		}
//		for (i=0;i<90;i++)
//		{
//		Servo2_Control(i);	
//		Servo1_Control(90-i);	
//		HAL_Delay(10);
//		}
//		X_Servo_Control(X_mid);
//		Y_Servo_Control(Y_mid);
		if(getTime10MsFlag())//注意，此时其实是5ms
		{

			if(ball==1)//有小球时，进行控制。
			{
			//	Change_mode(square_mode);//模式选择
		  //Change_mode(ceter_mode);//模式选择
				Change_mode(Mode);//模式选择
				X_Speed_Set = PID_Pos_Control(&X_POS_pid,X_Pos,X_Target);//位置外环，得到期望速度。
				Y_Speed_Set = PID_Pos_Control(&Y_POS_pid,Y_Pos,Y_Target);
				if (flag==1)
				{
				Last_X_Pos = X_Pos;
				Last_Y_Pos = Y_Pos;
			  flag =0;
				}
				
				X_Speed=(X_Pos-Last_X_Pos);
				Y_Speed=(Y_Pos-Last_Y_Pos);
//				X_PWM = PID_SPEED_Control(&X_SPEED_pid,X_Speed,0);//内环测试
//				Y_PWM = PID_SPEED_Control(&Y_SPEED_pid,Y_Speed,0);
				X_PWM = PID_SPEED_Control(&X_SPEED_pid,X_Speed,X_Speed_Set);
				Y_PWM = PID_SPEED_Control(&Y_SPEED_pid,Y_Speed,Y_Speed_Set);
//角度转换测试代码
//			Servo1_Control(0);
//			HAL_Delay(1000);
//			Servo1_Control(45);
//			HAL_Delay(1000);
//				Servo1_Control(90);
//			HAL_Delay(1000);
//			X_Servo_Control(1499);
				//距离小于10滤波
				if(((X_Pos-X_Target)<=8)&((-X_Pos+X_Target)<=8)&((Y_Pos-Y_Target)<=8)&((-Y_Pos+Y_Target)<=8))
				{
					X_PWM=0;
					Y_PWM=0;
					X_POS_pid.errsam=0;		
					Y_POS_pid.errsam=0;	
				}	
				out1=X_mid+X_PWM;
				out2=Y_mid-Y_PWM;
				X_Servo_Control(out1);//单是位置环
				Y_Servo_Control(out2);
				Last_X_Pos = X_Pos;
				Last_Y_Pos = Y_Pos;
				}
			else if (ball == 0)//没有小球时，板面保持水平
			{
				X_Servo_Control(X_mid);
				Y_Servo_Control(Y_mid);
				Last_X_Pos=150;
				Last_Y_Pos=150;
				X_Pos=150;
				Y_Pos=150;		
				X_POS_pid.errsam=0;		
				Y_POS_pid.errsam=0;	
				X_POS_pid.err=0;
				X_POS_pid.last_err=0;
				X_POS_pid.prev_err=0;
				Y_POS_pid.err=0;
				Y_POS_pid.last_err=0;
				Y_POS_pid.prev_err=0;
				flag=1;	
			}
			Data_Debug();//虚拟示波器
			data_count=0;//运行帧率
	  }

		if(getTimeSFlag())//运行指示灯
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
//			data_count=0;//运行帧率
		}
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Y_Servo_Control(float pwm)
{
	if (pwm>1499)//角度变换后，角度限制只能在90°到180°之间变换
		{
		pwm = 1499;
	}
	else if (pwm<499)
	{
		pwm = 499;
	}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t )(2998-pwm));//因为装置原因，将角度翻转，0°对应180°，45°对应135°，90°对应90°，按此关系类推。
	
}
void X_Servo_Control(float pwm)
{
	if (pwm>1499)
		{
		pwm = 1499;
	}
	else if (pwm<499)
	{
		pwm = 499;
	}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t )(2998-pwm));//因为装置原因，将角度翻转，0°对应180°，45°对应135°，90°对应90°，按此关系类推。
	
}
void Servo1_Control(uint16_t angle)
{
	
	/*舵机参数：f=50hz，t=20ms
	--------------------------
	|0   |45 |90 |135|180|   度|
	-------------------------
	|0.5 |1.0|1.5|2.0|2.5| 毫秒|
	-------------------------- 
	500       1500    2500
	*/
	float temp;
	temp = (100.0/9.0)*(180-angle)+500.0;
	if (temp>2499)
		{
		temp = 2499;
	}
	else if (temp<499)
	{
		temp = 499;
	}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, (uint16_t )temp);
	
}
void Servo2_Control(uint16_t angle)
{
	
	/*舵机参数：f=50hz，t=20ms
	--------------------------
	|0   |45 |90 |135|180|   度|
	-------------------------
	|0.5 |1.0|1.5|2.0|2.5| 毫秒|
	-------------------------- 
	500       1500    2500
	*/
	float temp;
	temp = (100.0/9.0)*(180-angle)+500.0;
	if (temp>2499)
		{
		temp = 2499;
	}
	else if (temp<499)
	{
		temp = 499;
	}
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t )temp);
	
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file
   */
    if(huart ->Instance == USART3)
   {
    openMVDataProess(RxByte);//串口数据处理
		while(HAL_UART_Receive_IT(&huart3,&RxByte,1)==HAL_OK);
	}
	 if(huart ->Instance == USART1)
   {
		HC05DataProess(RxByte);//串口数据处理
		while(HAL_UART_Receive_IT(&huart1,&RxByte,1)==HAL_OK);
	}
}


// 用在串口中断中处理接收到的openMV数据
// 通信格式是R(1200, 4300,ball,light)O 
//
float X_pos[100]={0},Y_pos[100]={0};
	int count=0;
void openMVDataProess(uint8_t RxByte)
{
	uint16_t vas = 0;
	uint16_t i = 0;
	// R 为起始标志
		 if(RxByte == 'R'){
			Rx_Count = 0;
		}
		 		if(Rx_Count>=254)
		{
			Rx_Count=-1;
		}
		RxBuff[Rx_Count++]=RxByte;
			if(RxBuff[0] == 'R' && RxByte == 'O'){
				data_count++;//计算帧率
		// 收到一帧数据开始处理
		if(RxBuff[1] == '('){
			i = 0;
			while(RxBuff[2 + i] != ','){
				vas = (RxBuff[2 + i] - '0') + vas * 10;
				i ++;
			}
			X_Pos = vas*5/4;//乘以一个系数因子
			vas = 0;
			while(RxBuff[4 + i] != ','){
				vas = (RxBuff[4 + i] - '0') + vas * 10;
				i ++;
			}
			Y_Pos = vas*5/4;//乘以一个系数因子
			vas = 0;
			while(RxBuff[6 + i] != ','){
			vas = (RxBuff[6 + i] - '0') + vas * 10;
			i ++;
			}
			ball = vas;
			vas = 0;
			//后面加激光再添加本段代码
			while(RxBuff[8 + i] != ','){
			vas = (RxBuff[8 + i] - '0') + vas * 10;
			i ++;
			}
			Light_X_Pos = vas*5/4;//乘以一个系数因子 
			vas = 0;
			while(RxBuff[10 + i] != ','){
			vas = (RxBuff[10 + i] - '0') + vas * 10;
			i ++;
			}
			Light_Y_Pos = vas*5/4;//乘以一个系数因子 
			vas = 0;
			while(RxBuff[12 + i] != ')'){
				vas = (RxBuff[12 + i] - '0') + vas * 10;
				i ++;
			}
			light = vas;
			vas = 0;
			
		}
		
//		if(count==10)
//		{
//			for(i=0;i<9;i++)
//			{
//			 X_Pos+=X_pos[i];
//			 Y_Pos+=Y_pos[i];
//			}
//			X_Pos=X_Pos/9;
//			Y_Pos=Y_Pos/9;
//			count=0;
//		}
	
	}
	
}	
/*HC-05蓝牙模块*/
void HC05DataProess(uint8_t RxByte)
{
	// R 为起始标志
		 if(RxByte == 0xA5){
			Rx_Count_hc = 0;
		}
		 		if(Rx_Count_hc>=254)
		{
			Rx_Count_hc=-1;
		}
		RxBuff_hc[Rx_Count_hc++]=RxByte;
		if(RxBuff_hc[0] == 0xA5 && RxByte == 0xAA){
			switch(RxBuff_hc[3])
		 {
				case 0xB2 : 					//注意前后左右移动之前回中
					Y_Target=Y_Target-30; Mode = Control_mode;
					break;
			  case 0xB4 :					
					X_Target=X_Target-30;Mode = Control_mode;
					break;
			  case 0xB6 : 					
					X_Target=X_Target+30;Mode = Control_mode;
					break;
			  case 0xB8 : 					
					Y_Target=Y_Target+30;Mode = Control_mode;
					break;		
			  case 0xB5 : Mode = ceter_mode;break;
//			  case 0xBA : Mode = square_mode; break;//长方形模式已经设置为方形，故这里不多余配置。
			  case 0xBB : Mode = cycle_mode; break;
			  case 0xBC : Mode = light_mode; break;			
			  case 0xBA : Mode = tangular_mode ; break;					 
		 }

		}
	
}	
//		//虚拟示波器

//画圆
double xcriclecos[360] = {
0.99985,0.99939,0.99863,0.99756,0.99619,0.99452,0.99255,0.99027,0.98769,0.98481,
0.98163,0.97815,0.97437,0.97030,0.96593,0.96126,0.95630,0.95106,0.94552,0.93969,
0.93358,0.92718,0.92050,0.91355,0.90631,0.89879,0.89101,0.88295,0.87462,0.86603,
0.85717,0.84805,0.83867,0.82904,0.81915,0.80902,0.79864,0.78801,0.77715,0.76604,
0.75471,0.74314,0.73135,0.71934,0.70711,0.69466,0.68200,0.66913,0.65606,0.64279,
0.62932,0.61566,0.60182,0.58779,0.57358,0.55919,0.54464,0.52992,0.51504,0.50000,
0.48481,0.46947,0.45399,0.43837,0.42262,0.40674,0.39073,0.37461,0.35837,0.34202,
0.32557,0.30902,0.29237,0.27564,0.25882,0.24192,0.22495,0.20791,0.19081,0.17365,
0.15643,0.13917,0.12187,0.10453,0.08716,0.06976,0.05234,0.03490,0.01745,0.00000,
-0.01745,-0.03490,-0.05234,-0.06976,-0.08716,-0.10453,-0.12187,-0.13917,-0.15643,-0.17365,
-0.19081,-0.20791,-0.22495,-0.24192,-0.25882,-0.27564,-0.29237,-0.30902,-0.32557,-0.34202,
-0.35837,-0.37461,-0.39073,-0.40674,-0.42262,-0.43837,-0.45399,-0.46947,-0.48481,-0.50000,
-0.51504,-0.52992,-0.54464,-0.55919,-0.57358,-0.58779,-0.60182,-0.61566,-0.62932,-0.64279,
-0.65606,-0.66913,-0.68200,-0.69466,-0.70711,-0.71934,-0.73135,-0.74314,-0.75471,-0.76604,
-0.77715,-0.78801,-0.79864,-0.80902,-0.81915,-0.82904,-0.83867,-0.84805,-0.85717,-0.86603,
-0.87462,-0.88295,-0.89101,-0.89879,-0.90631,-0.91355,-0.92050,-0.92718,-0.93358,-0.93969,
-0.94552,-0.95106,-0.95630,-0.96126,-0.96593,-0.97030,-0.97437,-0.97815,-0.98163,-0.98481,
-0.98769,-0.99027,-0.99255,-0.99452,-0.99619,-0.99756,-0.99863,-0.99939,-0.99985,-1.00000,
-0.99985,-0.99939,-0.99863,-0.99756,-0.99619,-0.99452,-0.99255,-0.99027,-0.98769,-0.98481,
-0.98163,-0.97815,-0.97437,-0.97030,-0.96593,-0.96126,-0.95630,-0.95106,-0.94552,-0.93969,
-0.93358,-0.92718,-0.92050,-0.91355,-0.90631,-0.89879,-0.89101,-0.88295,-0.87462,-0.86603,
-0.85717,-0.84805,-0.83867,-0.82904,-0.81915,-0.80902,-0.79864,-0.78801,-0.77715,-0.76604,
-0.75471,-0.74314,-0.73135,-0.71934,-0.70711,-0.69466,-0.68200,-0.66913,-0.65606,-0.64279,
-0.62932,-0.61566,-0.60182,-0.58779,-0.57358,-0.55919,-0.54464,-0.52992,-0.51504,-0.50000,
-0.48481,-0.46947,-0.45399,-0.43837,-0.42262,-0.40674,-0.39073,-0.37461,-0.35837,-0.34202,
-0.32557,-0.30902,-0.29237,-0.27564,-0.25882,-0.24192,-0.22495,-0.20791,-0.19081,-0.17365,
-0.15643,-0.13917,-0.12187,-0.10453,-0.08716,-0.06976,-0.05234,-0.03490,-0.01745,-0.00000,
0.01745,0.03490,0.05234,0.06976,0.08716,0.10453,0.12187,0.13917,0.15643,0.17365,
0.19081,0.20791,0.22495,0.24192,0.25882,0.27564,0.29237,0.30902,0.32557,0.34202,
0.35837,0.37461,0.39073,0.40674,0.42262,0.43837,0.45399,0.46947,0.48481,0.50000,
0.51504,0.52992,0.54464,0.55919,0.57358,0.58779,0.60182,0.61566,0.62932,0.64279,
0.65606,0.66913,0.68200,0.69466,0.70711,0.71934,0.73135,0.74314,0.75471,0.76604,
0.77715,0.78801,0.79864,0.80902,0.81915,0.82904,0.83867,0.84805,0.85717,0.86603,
0.87462,0.88295,0.89101,0.89879,0.90631,0.91355,0.92050,0.92718,0.93358,0.93969,
0.94552,0.95106,0.95630,0.96126,0.96593,0.97030,0.97437,0.97815,0.98163,0.98481,
0.98769,0.99027,0.99255,0.99452,0.99619,0.99756,0.99863,0.99939,0.99985,1.00000,
};

double ycriclesin[360] = {
0.01745,0.03490,0.05234,0.06976,0.08716,0.10453,0.12187,0.13917,0.15643,0.17365,
0.19081,0.20791,0.22495,0.24192,0.25882,0.27564,0.29237,0.30902,0.32557,0.34202,
0.35837,0.37461,0.39073,0.40674,0.42262,0.43837,0.45399,0.46947,0.48481,0.50000,
0.51504,0.52992,0.54464,0.55919,0.57358,0.58779,0.60182,0.61566,0.62932,0.64279,
0.65606,0.66913,0.68200,0.69466,0.70711,0.71934,0.73135,0.74314,0.75471,0.76604,
0.77715,0.78801,0.79864,0.80902,0.81915,0.82904,0.83867,0.84805,0.85717,0.86603,
0.87462,0.88295,0.89101,0.89879,0.90631,0.91355,0.92050,0.92718,0.93358,0.93969,
0.94552,0.95106,0.95630,0.96126,0.96593,0.97030,0.97437,0.97815,0.98163,0.98481,
0.98769,0.99027,0.99255,0.99452,0.99619,0.99756,0.99863,0.99939,0.99985,1.00000,
0.99985,0.99939,0.99863,0.99756,0.99619,0.99452,0.99255,0.99027,0.98769,0.98481,
0.98163,0.97815,0.97437,0.97030,0.96593,0.96126,0.95630,0.95106,0.94552,0.93969,
0.93358,0.92718,0.92050,0.91355,0.90631,0.89879,0.89101,0.88295,0.87462,0.86603,
0.85717,0.84805,0.83867,0.82904,0.81915,0.80902,0.79864,0.78801,0.77715,0.76604,
0.75471,0.74314,0.73135,0.71934,0.70711,0.69466,0.68200,0.66913,0.65606,0.64279,
0.62932,0.61566,0.60182,0.58779,0.57358,0.55919,0.54464,0.52992,0.51504,0.50000,
0.48481,0.46947,0.45399,0.43837,0.42262,0.40674,0.39073,0.37461,0.35837,0.34202,
0.32557,0.30902,0.29237,0.27564,0.25882,0.24192,0.22495,0.20791,0.19081,0.17365,
0.15643,0.13917,0.12187,0.10453,0.08716,0.06976,0.05234,0.03490,0.01745,0.00000,
-0.01745,-0.03490,-0.05234,-0.06976,-0.08716,-0.10453,-0.12187,-0.13917,-0.15643,-0.17365,
-0.19081,-0.20791,-0.22495,-0.24192,-0.25882,-0.27564,-0.29237,-0.30902,-0.32557,-0.34202,
-0.35837,-0.37461,-0.39073,-0.40674,-0.42262,-0.43837,-0.45399,-0.46947,-0.48481,-0.50000,
-0.51504,-0.52992,-0.54464,-0.55919,-0.57358,-0.58779,-0.60182,-0.61566,-0.62932,-0.64279,
-0.65606,-0.66913,-0.68200,-0.69466,-0.70711,-0.71934,-0.73135,-0.74314,-0.75471,-0.76604,
-0.77715,-0.78801,-0.79864,-0.80902,-0.81915,-0.82904,-0.83867,-0.84805,-0.85717,-0.86603,
-0.87462,-0.88295,-0.89101,-0.89879,-0.90631,-0.91355,-0.92050,-0.92718,-0.93358,-0.93969,
-0.94552,-0.95106,-0.95630,-0.96126,-0.96593,-0.97030,-0.97437,-0.97815,-0.98163,-0.98481,
-0.98769,-0.99027,-0.99255,-0.99452,-0.99619,-0.99756,-0.99863,-0.99939,-0.99985,-1.00000,
-0.99985,-0.99939,-0.99863,-0.99756,-0.99619,-0.99452,-0.99255,-0.99027,-0.98769,-0.98481,
-0.98163,-0.97815,-0.97437,-0.97030,-0.96593,-0.96126,-0.95630,-0.95106,-0.94552,-0.93969,
-0.93358,-0.92718,-0.92050,-0.91355,-0.90631,-0.89879,-0.89101,-0.88295,-0.87462,-0.86603,
-0.85717,-0.84805,-0.83867,-0.82904,-0.81915,-0.80902,-0.79864,-0.78801,-0.77715,-0.76604,
-0.75471,-0.74314,-0.73135,-0.71934,-0.70711,-0.69466,-0.68200,-0.66913,-0.65606,-0.64279,
-0.62932,-0.61566,-0.60182,-0.58779,-0.57358,-0.55919,-0.54464,-0.52992,-0.51504,-0.50000,
-0.48481,-0.46947,-0.45399,-0.43837,-0.42262,-0.40674,-0.39073,-0.37461,-0.35837,-0.34202,
-0.32557,-0.30902,-0.29237,-0.27564,-0.25882,-0.24192,-0.22495,-0.20791,-0.19081,-0.17365,
-0.15643,-0.13917,-0.12187,-0.10453,-0.08716,-0.06976,-0.05234,-0.03490,-0.01745,-0.00000,

};
void line(float set_x1,float set_y1,float set_x2,float set_y2)
{
		const  float priod = 1500.0;      //周期(毫秒)
  	static uint32_t MoveTimeCnt = 0;  //计时变量
  	float  Normalization = 0.0;       //累加值比周期值的数
	  float  set_x,set_y;
    MoveTimeCnt += 5;	
	  Normalization = (float)MoveTimeCnt / priod;	 //对板球周期归一化，相当于把一条直线上设置多个目标点，不断叠加到达最终目标点。
	  if(Normalization>=1.0)
		{
		 Normalization = 1.0;
		}
    X_Target = set_x1 + ((set_x2-set_x1) * Normalization) ;
		Y_Target = set_y1 + ((set_y2-set_y1) * Normalization) ;	
		X_Speed_Set = PID_Pos_Control(&X_POS_pid,X_Pos,X_Target);//位置外环，得到期望速度。
		Y_Speed_Set = PID_Pos_Control(&Y_POS_pid,Y_Pos,Y_Target);
		X_Servo_Control(X_mid+X_Speed_Set);
		Y_Servo_Control(Y_mid-Y_Speed_Set);
}
/*画长方形*/
void tangular (float set_x1,float set_y1,float set_x2,float set_y2,float set_x3,float set_y3,float set_x4,float set_y4)
{
	  const  float priod = 100.0;  //周期(毫秒)
  	static uint32_t MoveTimeCnt = 0;
  	float  Normalization = 0.0;
    float  set_x,set_y;
	  float  hc_time = 800;                          //停留过渡时间
    MoveTimeCnt += 20;							                 //每5ms运算1次
	
	if(MoveTimeCnt<=priod)
	{
		
		 Normalization = (float)MoveTimeCnt / priod;	 //对板球周期归一化
	   if(Normalization>=1.0)
		 {
		  Normalization = 1.0;
		 }
    X_Target = set_x1 + ((set_x2-set_x1) * Normalization) ;
		Y_Target = set_y1 + ((set_y2-set_y1) * Normalization) ;	
	}
  else if((MoveTimeCnt>(priod+hc_time))&&(MoveTimeCnt<(2*priod+2*hc_time)))
		{
			Normalization = (float)(MoveTimeCnt-(priod+hc_time)) / priod;	 //对板球周期归一化
	   if(Normalization>=1.0)
		 {
		  Normalization = 1.0;
		 }
		X_Target = set_x2 + ((set_x3-set_x2) * Normalization) ;
		Y_Target = set_y2 + ((set_y3-set_y2) * Normalization) ;	
		}
		  else if(MoveTimeCnt>(2*priod+2*hc_time)&&(MoveTimeCnt<(3*priod+3*hc_time)))
		{
			Normalization = (float)(MoveTimeCnt-(2*priod+2*hc_time)) / priod;	 //对板球周期归一化
	   if(Normalization>=1.0)
		 {
		  Normalization = 1.0;
		 }
		X_Target = set_x3 + ((set_x4-set_x3) * Normalization) ;
		Y_Target = set_y3 + ((set_y4-set_y3) * Normalization) ;	
		}
		else if(MoveTimeCnt>(3*priod+3*hc_time)&&(MoveTimeCnt<(4*priod+4*hc_time)))
		{
			Normalization = (float)(MoveTimeCnt-(3*priod+3*hc_time)) / priod;	 //对板球周期归一化
	   if(Normalization>=1.0)
		 {
		  Normalization = 1.0;
		 }
		X_Target = set_x4 + ((set_x1-set_x4) * Normalization) ;
		Y_Target = set_y4 + ((set_y1-set_y4) * Normalization) ;	
		}
	 else if(MoveTimeCnt>(4*priod+4*hc_time))
	 {MoveTimeCnt=0;}
		
}  
double real_radiu=0;
float R_out=0;
void circle_move(double radiu)
{
	static double outradiu = 1;//设置输出圆
	static int criclecount = 0;
	float R_out=0;
	real_radiu=sqrt((X_Pos - X_Center)*(X_Pos - X_Center) + (Y_Pos - Y_Center)*(Y_Pos - Y_Center));
	R_out=PID_R_MOVE(&R_MOVE_pid,real_radiu,radiu);
	outradiu=outradiu+R_out;
	if (outradiu >= 150)outradiu = 150;//对球运动的半径进行限制,初始设定值为半径100.
	if (outradiu <= 0)outradiu = 1;
	X_Target = xcriclecos[criclecount] * outradiu + X_Center;//圆周运动时得到预期的坐标目标值。
	Y_Target = ycriclesin[criclecount] * outradiu + Y_Center;
	//outradiu=outradiu+PID   返回值计算 后面添加圆PID环
	criclecount += 5;//调节一次步进的角度，控制圆周运动速度
	if (criclecount >= 360)criclecount = 0;
}

unsigned char Wait_flag = 1;//在蓝牙传输时改变此变量为0，后续看是否有必要。
/*画正方形，固定点*/
void square(void)
{
			static unsigned char flag1=255,flag2;//加上蓝牙传输时后改变此变量后删除"=255"
			if(Wait_flag==0){flag1++;X_Target=50;X_Target=50;flag2=0;}//等待一段时间到达初始目标点，flag1 代表计数器
			if(flag1==255){flag1=0;Wait_flag=1;flag2=1;}	
			
			if(flag2==5) //第五段，沿着第四条边移动
			{
				X_Target+=10;	 if(X_Target>=250)flag2=2;			
			}
			if(flag2==4) //第四段，沿着第三条边移动
			{
				Y_Target-=10;	if(Y_Target<=50)flag2=5;			
			}
			if(flag2==3) //第三段，沿着第二条边移动
			{
				X_Target-=10;	if(X_Target<=50)flag2=4;			
			}
			if(flag2==2) //第二段，沿着第一条边移动
			{
				Y_Target+=10;	if(Y_Target>=250)flag2=3;		
			}
			if(flag2==1) //第一段，沿着X轴正方向到达正方形边
			{
				X_Target+=10;	Y_Target=50; if(X_Target>=250)flag2=2;	//循环	
			}
}
/*蓝牙传输模式*/


/*执行不同的模式*/
void Change_mode(unsigned short mode )
{
	if(mode == cycle_mode){X_POS_Ki=Y_POS_Ki=0;}//画圆时内环不要积分，取消掉位置环积分。
  else {X_POS_Ki=Y_POS_Ki=0.001;}
	switch(mode)
		 {
			  case ceter_mode :
					X_Target=X_Center;
					Y_Target=Y_Center; 
					break;
			  case square_mode : square(); break;
			  case light_mode : X_Target = Light_X_Pos;Y_Target=Light_Y_Pos;  break;
			  case cycle_mode : circle_move(100); break;			
			  case tangular_mode :tangular (50,50,250,50,250,250,50,250) ; break;	
				case Control_mode: ;break;
			
		 }
		 
}		 
//
void Data_Debug(void)
{
	unsigned char Send_Count = 0;
	unsigned char i = 0;
	DataScope_Get_Channel_Data( X_Target , 1); //将小球X坐标设定值    写入通道 1
	DataScope_Get_Channel_Data( X_Pos, 2 ); //将实际小球X坐标  写入通道 2
	DataScope_Get_Channel_Data( Y_Target , 3); //将小球Y坐标设定值    写入通道 3
	DataScope_Get_Channel_Data( Y_Pos, 4 ); //将实际小球X坐标  写入通道 4
	DataScope_Get_Channel_Data( X_Speed , 5); //将小球速度    写入通道 5/6
	DataScope_Get_Channel_Data( Y_Speed, 6 ); 
	DataScope_Get_Channel_Data( real_radiu , 7); //将小球速度    写入通道 5/6
	DataScope_Get_Channel_Data( data_count, 8 ); 
	Send_Count = DataScope_Data_Generate(8); //生成6个通道的 格式化帧数据，返回帧数据长度
	HAL_UART_Transmit_IT(&huart1, (uint8_t*)DataScope_OutPut_Buffer, Send_Count);//串口发送数据
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
