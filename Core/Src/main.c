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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MotorControl.h"
#include "ControlTask.h"
#include "PID.h"
#include "gongxun.h"
#include "lineFollowSensor.h"
#include "user_usart.h"
#include <stdio.h>
//#include "Max7219.h"
#include "uart234.h"
//#include "ssd1306.h"
#include "gongxun.h"
#include "Buzzer/buzzerDriver.h"

#include "MAX7219/max7219_matrix.h"

#include "letter_shell/src/shell_port.h"

#include "SCSLib/uart.h"

#include "SCS_servo/SCS_servo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int32_t y_speed=0;
int32_t x_speed=0;
int32_t a_speed=0;
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_INT), xsp, &x_speed, x_speed);
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_INT), ysp, &y_speed, y_speed);
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_INT), asp, &a_speed, a_speed);

int32_t cx_main=0;

/*
存放障碍物的数组，地图格点5×8
确定起始位置
当前小车所在的XY坐标
此次小车的起始坐标和终点坐标
*/
int32_t obs[5][8]={0};
int32_t START_MODE=1;//取左下角起始为0，右下角起始为1
int32_t x_position=0;
int32_t y_position=0;
int32_t BEGIN_X=0;
int32_t BEGIN_Y=0;
int32_t END_X=0;
int32_t END_Y=0;

uint8_t startFlag=0;
SHELL_EXPORT_VAR(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_VAR_CHAR), stf, &startFlag, startFlag);

extern int32_t motorspeed_set[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void led_shan(void);
static void MX_USART2_UART_Init(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	led_shan();
	led_shan();
	led_shan();
	led_shan();
	led_shan();
	led_shan();

	
	//打印初始化指示语句
	user_main_printf("\n\rInitating...");
	
	//开启控制舵机的串口2接收中断
	HAL_UART_Receive_IT(&huart5, (uint8_t*)data_one_byte, 1);
	//初始化舵机控制结构体
	ArmInit();
//	readF2ram();
	//舵机位置归中
	ArmGoMiddle();
	
	//初始化LED点阵显示
	MAX7219_MatrixInit(&hspi1, MAX7219_CS_GPIO_Port, MAX7219_CS_Pin);
	MAX7219_MatrixUpdate();
	uint8_t colororder[]={0,0,0,0,0,0};
	MAX7219_mywrite(colororder);
	MAX7219_MatrixUpdate();
	user_main_printf("LED Matrix ok...");

	//开启定时器的编码器模式
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	user_main_printf("Encoder Started...");
	
	//开启8路PWM输出-控制四个电机用
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);	
	user_main_printf("Motor PWM Started...");

	//初始化PID结构体参数
	User_PID_Init(&udPIDParameter0);
	User_PID_Init(&udPIDParameter1);
	User_PID_Init(&udPIDParameter2);
	User_PID_Init(&udPIDParameter3);
	User_PID_Init(&udPIDParameter3);
	User_PID_Init(&xAnglePIDParameter4);
	User_PID_Init(&xPositionPIDParameter5);
	User_PID_Init(&yAnglePIDParameter6);
	User_PID_Init(&yPositionPIDParameter7);
	LineFollowInit();
	user_main_printf("PID parameter Initiated...");
	
	//定时器6溢出中断频率为1kHz
	HAL_TIM_Base_Start_IT(&htim6);
	user_main_printf("【TIM6 interrupt】 Started...(1kHz)");
	
	user_main_printf("Enjoy a music...");
	//开启蜂鸣器PWM输出
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	
	//初始化letter-shell
	User_Shell_Init();
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&recv_buf, 1);

#if 1
while(1)
{
	if(startFlag){
		startFlag=0;
		tim6enable=1;
		AngleAndPositionTIM=1;
		ManufacturingProcesses();
	}
	if(!HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin))
		startFlag=1;
	led_shan();
}
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	user_main_printf("begin------------------------");
  while (1)
  {
		
		led_shan();
//		user_main_printf("motor_speed 0-3 zh zq yq yh: %d,%d,%d,%d",motorspeed[0],motorspeed[1],motorspeed[2],motorspeed[3]);
		GetSensorData();
		printf("OUT5:%d \t",Sensor_JG_Buffer[0]);
		printf("OUT6:%d \t",Sensor_JG_Buffer[1]);
		printf("OUT7:%d \t",Sensor_JG_Buffer[2]);
		printf("OUT8:%d \t",Sensor_JG_Buffer[3]);
		printf("OUT9:%d \t",Sensor_JG_Buffer[4]);
		printf("OUT10:%d\t",Sensor_JG_Buffer[5]);
		printf("OUT11:%d\t",Sensor_JG_Buffer[6]);
		printf("OUT12:%d\t",Sensor_JG_Buffer[7]);
		printf("OUT1:%d \t",Sensor_JG_Buffer[8]);
		printf("OUT2:%d \t",Sensor_JG_Buffer[9]);
		printf("OUT3:%d \t",Sensor_JG_Buffer[10]);
		printf("OUT4:%d \t",Sensor_JG_Buffer[11]);
		printf("OUT13:%d\t",Sensor_JG_Buffer[12]);
		printf("SJ1:%d \t",Sensor_JG_Buffer[13]);
		printf("OUT15:%d\t",Sensor_JG_Buffer[14]);
		printf("OUT16:%d\n\r",Sensor_JG_Buffer[15]);
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void led_shan(){
	LED1_ON;
	HAL_Delay(200);
	LED1_OFF;
	HAL_Delay(200);
}

static void MX_USART2_UART_Init()
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

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
