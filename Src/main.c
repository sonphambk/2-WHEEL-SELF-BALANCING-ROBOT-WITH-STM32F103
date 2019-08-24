/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "math.h"
#include "sd_hal_mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define 	dt1  0.01
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void bias(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint16_t gx[20];
SD_MPU6050 mpu1;
float sum_gForcex = 0;
float sum_gForcey = 0;
float sum_gForcez = 0;
float gForcex = 0;
float gForcey = 0;
float gForcez = 0;
float ax_off = 0.0306123048;
float ay_off = -0.00247998047;
float az_off = 1.00272071;
float gx_off = 376.86;
float gy_off = 93.00;
float gz_off = -92.64;
float rotx = 0;
float roty = 0;
float rotz = 0;
float bias_rotx = 0;
float bias_roty = 0;
float bias_rotz = 0;
float angle_roll = 0;
float angle_pitch = 0;
float P_part1 ,I_part1,D_part1,Error,pre_Error,pre_pre_Error,Out1,pre_Out1;
float  P_part2 ,I_part2,D_part2,Out2,pre_Out2;
float  Kp1 = 25; //28 //30 //26
float  Ki1 = 420;//230//350//390
float  Kd1 = 1.2;//1.14//1.18//1.18
float maxPID = 255;
char receive_buff[30];
float setpoint = -2;  
float vel2;
float vel;
float pitch;
float roll;
float g_x;
float g_y;
float g_z;
float a_x;
float a_y;
float a_z;
float left_PWM;
float right_PWM;
float vel_left;
float vel_right;
volatile short encoder_out_left =0,encoder_out_pre =0;
volatile short encoder_out_right=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void move(float motor,float dir)
	{
		 if(motor == 0)
      {
					if(dir == 0)
						{
								HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
						}
					else 
						{
								HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
						}
			}
		 else if (motor == 1)
      {			
					if(dir == 0)
						{
								HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
						}
					else 
						{
								HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
						}
			 }
			else if (motor == 2)
				{
							HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
							__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
							__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);
				}
	}
void PID(float setpoint,float angle_current)
	{
		
			if (angle_roll >-30 && angle_roll <30)
				{
//					if (receive_buff[0] == 'F')
//						{
//								setpoint = -3.5;
//								Error = setpoint - angle_current;
//								P_part1 = Kp1*(Error - pre_Error);
//								I_part1 = 0.5*Ki1*dt1*(Error + pre_Error);
//								D_part1= Kd1/dt1*( Error - 2*pre_Error+ pre_pre_Error);
//								Out1 = pre_Out1 + P_part1 + I_part1 + D_part1 ;
//								if (Out1 > maxPID) Out1 = maxPID;
//								else if (Out1 <-maxPID) Out1 = -maxPID;
//						if (Out1 >0)
//						{
//							move(0,1);
//							move(1,1);
//							vel= Out1;
//							if (vel >10 && vel <100 ) vel = 100;
//							vel_left = vel;
//							vel_right = vel;
//							__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,vel_left);
//							__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,vel_right);
//						}							
//					else if (Out1<0)
//						{
//							move(0,0);
//							move(1,0);
//							vel= Out1;
//							if (vel >-70 && vel <-100 ) vel = - 100;
//							vel_left = vel;
//							vel_right = vel;
//							__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(vel_left*-1));
//							__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(vel_right*-1));
//						}
//					}
//				else if (receive_buff[0] =='B')
//					{
//							 setpoint = 2.5;
//								Error = setpoint - angle_current;
//								P_part1 = Kp1*(Error - pre_Error);
//								I_part1 = 0.5*Ki1*dt1*(Error + pre_Error);
//								D_part1= Kd1/dt1*( Error - 2*pre_Error+ pre_pre_Error);
//								Out1 = pre_Out1 + P_part1 + I_part1 + D_part1 ;
//								if (Out1 > maxPID) Out1 = maxPID;
//								else if (Out1 <-maxPID) Out1 = -maxPID;
//								if (Out1 >0)
//									{
//										move(0,1);
//										move(1,1);
//										vel= Out1;
//										if (vel >10 && vel <100 ) vel = 100;
//										vel_left = vel;
//										vel_right = vel;
//										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,vel_left);
//										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,vel_right);
//									}							
//								else if (Out1<0)
//									{
//										move(0,0);
//										move(1,0);
//										vel= Out1;
//										if (vel >-70 && vel <-100 ) vel = - 100;
//										vel_left = vel;
//										vel_right = vel;
//										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(vel_left*-1));
//										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(vel_right*-1));
//									}
//						}
//				else if (receive_buff[0] == 'L')
//					{
//								setpoint = 0.5;
//								Error = setpoint - angle_current;
//								P_part1 = Kp1*(Error - pre_Error);
//								I_part1 = 0.5*Ki1*dt1*(Error + pre_Error);
//								D_part1= Kd1/dt1*( Error - 2*pre_Error+ pre_pre_Error);
//								Out1 = pre_Out1 + P_part1 + I_part1 + D_part1 ;
//								if (Out1 > maxPID) Out1 = maxPID;
//								else if (Out1 <-maxPID) Out1 = -maxPID;
//								if (Out1 >0)
//									{
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
//										vel= Out1;
//										if (vel >10 && vel <100 ) vel =100;
//										vel_left = vel;
//										vel_right = vel;
//										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,120);//vel_left-turnspeed
//										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,120);//vel_right+turnspeed
//									}							
//								else if (Out1<0)
//									{
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
//										vel= Out1;
//										if (vel >-100 && vel <-10 ) vel = - 100;
//										vel_left = vel;
//										vel_right = vel;
//										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,120);//(vel_left*-1)-turnspeed
//										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,120);//(vel_right*-1)+turnspeed
//									}
//					}
//				else if (receive_buff[0] == 'R')
//					{
//								setpoint = 0.5;
//								Error = setpoint - angle_current;
//								P_part1 = Kp1*(Error - pre_Error);
//								I_part1 = 0.5*Ki1*dt1*(Error + pre_Error);
//								D_part1= Kd1/dt1*( Error - 2*pre_Error+ pre_pre_Error);
//								Out1 = pre_Out1 + P_part1 + I_part1 + D_part1 ;
//								if (Out1 > maxPID) Out1 = maxPID;
//								else if (Out1 <-maxPID) Out1 = -maxPID;
//								if (Out1 >0)
//									{
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
//										vel= Out1;
//										if (vel >10 && vel <100 ) vel = 100;
//										vel_left = vel;
//										vel_right = vel;
//										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,120);
//										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,120);
//									}							
//								else if (Out1<0)
//									{
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
//										HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
//										vel= Out1;
//										if (vel >-100 && vel <-10 ) vel = - 100;
//										vel_left = vel;
//										vel_right = vel;
//										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,120);
//										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,120);
//									}
//					}
//				else
//					{
								setpoint = -2;
								Error = setpoint - angle_current;
								P_part1 = Kp1*(Error - pre_Error);
								I_part1 = 0.5*Ki1*dt1*(Error + pre_Error);
								D_part1= Kd1/dt1*( Error - 2*pre_Error+ pre_pre_Error);
								Out1 = pre_Out1 + P_part1 + I_part1 + D_part1 ;
								if (Out1 > maxPID) Out1 = maxPID;
								else if (Out1 <-maxPID) Out1 = -maxPID;
								if (Out1 >0)
									{
										move(0,1);
										move(1,1);
										vel= Out1;
									if (vel >10 && vel <70 ) vel = 70;
										vel_left = vel;
										vel_right = vel;
										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,vel_left);
										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,vel_right);
									}							
								else if (Out1<0)
									{
										move(0,0);
										move(1,0);
										vel= Out1;
										if (vel >-70 && vel <-0 ) vel = -70;
										vel_left = vel;
										vel_right = vel;
										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(vel_left*-1));
										__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,(vel_right*-1));
									}
					}
				
//			}
		else move(2,0);
 		pre_pre_Error = pre_Error;
    pre_Error = Error;
    pre_Out1 = Out1;	
		pre_Out2 = Out2;
		
	}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
			
		SD_MPU6050_Result result ;
		if (htim->Instance == TIM4 )
			{
		result = SD_MPU6050_Init(&hi2c1,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_250s );
		SD_MPU6050_ReadAll(&hi2c1,&mpu1);
//	  SD_MPU6050_ReadGyroscope(&hi2c1,&mpu1);
	  int16_t g_x = mpu1.Gyroscope_X;
	  int16_t g_y = mpu1.Gyroscope_Y;
	  int16_t g_z = mpu1.Gyroscope_Z;
    /* USER CODE BEGIN 3 */
//		SD_MPU6050_ReadAccelerometer(&hi2c1,&mpu1);
	  int16_t a_x = mpu1.Accelerometer_X;
	  int16_t a_y = mpu1.Accelerometer_Y;
	  int16_t a_z = mpu1.Accelerometer_Z;
 
		rotx = (float)(g_x - gx_off)/131;
		roty = (float)(g_y - gy_off)/131;
		rotz = (float)(g_z - gz_off)/131;

		gForcex = (float)(a_x - ax_off)/16384 ;
		gForcey = (float)(a_y - ay_off)/16384;
		gForcez = (float)(a_z - az_off)/16384 ;
		 
		 roll  = atan2((double)gForcey,(double)gForcez)*57.29577951;
		 pitch = -atan2((double)gForcex,(double)sqrt((double)gForcey*(double)gForcey+(double)gForcez*(double)gForcez))*57.29577951;
	 	 angle_roll  = 0.988*(angle_roll  + rotx*dt1) +0.012*roll;
		 angle_pitch = 0.988*(angle_pitch + roty*dt1) +0.012*pitch;
		 PID(setpoint,angle_roll);
		
			} 
	}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//	{
//		if (htim->Instance == TIM2)
//			{
//						encoder_out_left  = __HAL_TIM_GET_COUNTER(&htim2);
//			}
//	  if (htim->Instance == TIM3)
//			{
//						encoder_out_right =	__HAL_TIM_GET_COUNTER(&htim3);
//			}
//	}


/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
		SD_MPU6050_Result result ;
	  
		char* mpu_ok = {"MPU WORK FINE\n"};
		char* mpu_not = {"MPU NOT WORKING\n"};
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */



		HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
  MX_USART3_UART_Init();
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
//	HAL_TIM_Encoder_Start_IT(&htim2,TIM_CHANNEL_1|TIM_CHANNEL_2);
//	HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_1|TIM_CHANNEL_2);
	
	
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
//		encoder_out_left  = __HAL_TIM_GET_COUNTER(&htim2);
//		encoder_out_right = __HAL_TIM_GET_COUNTER(&htim3); 
//		
//			}
		/* USER CODE END WHILE */
		
  }
  /* USER CODE END 3 */
}
_ARMABI int fputc(int c, FILE * stream)/////////////////////////////////////////
	{
		HAL_UART_Transmit(&huart3,(uint8_t*)&c,1,100);
		return 0;
	}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
