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
#include "Max7219.h"
#include "uart234.h"
#include "ssd1306.h"
#include "gongxun.h"
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
	
	user_main_printf("Initating...");
	
	//初始化数码管显示
	Init_MAX7219();
	HAL_Delay(100);
	Write_Max7219(0x0f, 0x00);       //显示测试：1；测试结束，正常显示：0
	HAL_Delay(100);
	WriteClear_Max7219();
//	WriteNum_Max7219("123--213");
	user_main_printf("7segLED ok...");
	
	ssd1306_Init(&hi2c1);
  HAL_Delay(100);
  ssd1306_Fill(White);
	ssd1306_SetCursor(10, 20);
  ssd1306_WriteString("UP/DOWN", Font_16x26, Black);
  ssd1306_UpdateScreen(&hi2c1);
  HAL_Delay(100);
	user_main_printf("0.91OLED ok...");
	
	//测试并初始化机械臂位置
	Uart2_servoCtr(1);
//	while(SW2==UNPRESSED);
//	led_shan();
//	led_shan();
//	led_shan();
//	Uart2_servoCtr(2);
//	while(SW2==UNPRESSED);
//	led_shan();
//	led_shan();
//	led_shan();
//	Uart2_servoCtr(3);
//	while(SW2==UNPRESSED);
//	led_shan();
//	led_shan();
//	led_shan();
//	Uart2_servoCtr(4);
//	while(SW2==UNPRESSED);
//	led_shan();
//	led_shan();
//	led_shan();
//	Uart2_servoCtr(5);
//	while(SW2==UNPRESSED);
//	led_shan();
//	led_shan();
//	led_shan();
	
	user_main_printf("Robotic arm ok...");

//	user_main_printf("Serching QRcode.");
//	Uart3_readQRcode();
//	user_main_printf("Get it,the result is shown on the 7seg display.");
//	user_main_printf("Serching Color blocks.");
//	Uart3_readColor();
//	user_main_printf("Get it,the result is shown on 0.91'OLED.");
	
	//开启四个编码器
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	user_main_printf("Encoder Started...");
	
	//测试四个电机
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);	
	
	user_main_printf("Motor PWM Started...");
	

	User_PID_Init(&udPIDParameter0);
	User_PID_Init(&udPIDParameter1);
	User_PID_Init(&udPIDParameter2);
	User_PID_Init(&udPIDParameter3);
	User_PID_Init(&udPIDParameter3);
	User_PID_Init(&xAnglePIDParameter4);
	User_PID_Init(&xPositionPIDParameter5);
	User_PID_Init(&yAnglePIDParameter6);
	User_PID_Init(&yPositionPIDParameter7);
	user_main_printf("PID parameter Initiated...");
	LineFollowInit();
	user_main_printf("line sensor Started...");
	
	//定时器6溢出中断频率为1kHz
	HAL_TIM_Base_Start_IT(&htim6);
	user_main_printf("【TIM6 interrupt】 Started...(1kHz)");
	

	//for test 2021-1-3-21:20 by mxy	
//	lockFlag=1;
//	while(1){
//		while(SW2==UNPRESSED);
//		led_shan();
//		Uart2_servoCtr(9);
//		HAL_Delay(5000);
//		Uart2_servoCtr(14);
//	}

	ManufacturingProcesses();
	//for test 2021-1-2-20:53 by mxy	
	//for test 2021-1-3-16:16 by mxy	
	//for test 2021-1-5-16:45 by mxy	
	while(1){}
//	char temp__=3;
	OneGrid(FRONT,-15);
	OneGrid_sp(LEFT,FRONT,0);
	OneGrid(FRONT,2);
	Uart3_readQRcode();
	OneGrid_sp(LEFT,FRONT,0);
	OneGrid(FRONT,0);
	OneGrid(FRONT,0);
	OneGrid(FRONT,10);
	Uart3_readColor();
	Uart2_servoCtr(2);
	OneGrid(FRONT,-15);
	Grid_Lock();
	HAL_Delay(2000);
//	for(temp__=0;temp__<6;temp__++)
//	{
//		Uart2_servoCtr(3+temp__);
//		HAL_Delay(Catch1Time[temp__]);
//	}
	Uart2_servoCtr(3);
	HAL_Delay(Catch1Time[0]);
	ni(10);
	HAL_Delay(1000);
	Grid_UnLock();
	OneGrid(FRONT,0);
	OneGrid(FRONT,-15);
//	CarMovingTo=1;
	Grid_Lock();
	HAL_Delay(1000);
	Uart2_servoCtr(10);
	HAL_Delay(4500);
	Grid_UnLock();
	OneGrid(BACK,0);
	OneGrid(BACK,-15);
	ni(-10);
	Grid_Lock();
	
	Grid_Lock();
	Uart2_servoCtr(4);
	HAL_Delay(Catch1Time[1]);
	ni(10);
	HAL_Delay(1000);
	Grid_UnLock();
	OneGrid(FRONT,0);
	OneGrid(FRONT,-15);
	Grid_Lock();
	HAL_Delay(1000);
	Uart2_servoCtr(11);
	HAL_Delay(6000);
	Grid_UnLock();
	OneGrid(BACK,0);
	OneGrid(BACK,-15);
	ni(-10);
	Grid_Lock();
	
	Grid_Lock();
	Uart2_servoCtr(5);
	HAL_Delay(Catch1Time[2]);
	ni(10);
	HAL_Delay(1000);
	Grid_UnLock();
	OneGrid(FRONT,0);
	OneGrid(FRONT,-15);
	Grid_Lock();
	HAL_Delay(1000);
	Uart2_servoCtr(12);
	HAL_Delay(6000);
//	Grid_UnLock();
//	OneGrid(BACK,0);
//	OneGrid(BACK,-15);
//	ni(-10);
	Grid_Lock();
	
	
	//for test 2021-1-2-17:06 by lyj
//	OneGrid(0,-15);//step 1:front
//	AngleAndPositionTIM=0;
//	y_speed=-userSpeed_y;
//  while((HAL_GPIO_ReadPin(OUT3_Port,OUT3_Pin)||HAL_GPIO_ReadPin(OUT6_Port,OUT6_Pin))==0){}//start status is not being on grid
//	CarMovingTo = 3;
//	y_speed=0;
//	AngleAndPositionTIM=1;
//	AnglePosControl(0);
//	HAL_Delay(20);
//	AnglePosControl(1);
//	HAL_Delay(20);
//	AnglePosControl(2);
//	HAL_Delay(20);
//	AnglePosControl(3);
//	HAL_Delay(20);	
//	AnglePosControl(0);
//	HAL_Delay(20);
//	AnglePosControl(1);
//	HAL_Delay(20);
//	AnglePosControl(2);
//	HAL_Delay(20);
//	y_speed=0;
//	x_speed=0;//step 2:left
//	OneGrid(0,-15);
//	OneGrid(0,-15);
	while(1){led_shan();}//stop here
		
	while(1){
//
//		while(SW2==UNPRESSED);
		led_shan();
		OneGrid(0,0);
		OneGrid(0,0);
		OneGrid(0,0);
		OneGrid(0,-15);
		OneGrid(1,0);
		OneGrid(1,0);
		OneGrid(1,0);
		OneGrid(1,0);
		OneGrid(2,0);
		OneGrid(2,0);
		OneGrid(2,0);
		OneGrid(2,-15);
		OneGrid(3,0);
		OneGrid(3,0);
		OneGrid(3,0);
		OneGrid(3,0);
	}
//	OnTheWay(0,1);
	user_main_printf("OK, press sw2 to start...");
	while(SW2==UNPRESSED);
	led_shan();
	led_shan();
	led_shan();
	
	while(1){
//	user_main_printf("Serching QRcode.");
//	Uart3_readQRcode();
//	user_main_printf("Get it,the result is shown on the 7seg display.");
//	user_main_printf("Serching Color blocks.");
//	Uart3_readColor();
//	user_main_printf("Get it,the result is shown on 0.91'OLED.");
//		OneGrid(0,0);
//		OneGrid(0,0);
//		OneGrid(0,-20);
//		CarMovingTo=1;
//		HAL_Delay(1000);
//		OneGrid(1,0);
//		OneGrid(1,0);
//		OneGrid(1,0);
//		CarMovingTo=2;
//		HAL_Delay(1000);
//		OneGrid(2,0);
//		OneGrid(2,0);
//		OneGrid(2,-20);
//		CarMovingTo=3;
//		HAL_Delay(1000);
//		OneGrid(3,0);
//		OneGrid(3,0);
//		OneGrid(3,0);
//		CarMovingTo=0;
//		OneGrid(1,0);
//		OneGrid(1,0);
//		OneGrid(1,0);
//		OneGrid(1,0);
		HAL_Delay(1000);

		while(SW2==UNPRESSED){led_shan();};
	}	
//	OneGrid(0,0);
//	OneGrid(0,0);
//	OneGrid(0,0);
//	OneGrid(0,-0.6*userSpeed);
//	HAL_Delay(1000);
//	OneGrid(1,0);
//	OneGrid(1,0);
//	OneGrid(1,0);
//	OneGrid(1,-0.6*userSpeed);
//	HAL_Delay(1000);
//	OneGrid(2,0);
//	OneGrid(2,0);
//	OneGrid(2,0);
//	OneGrid(2,-0.6*userSpeed);
//	HAL_Delay(1000);
//	OneGrid(3,0);
//	OneGrid(3,0);
//	OneGrid(3,0);
//	OneGrid(3,-0.6*userSpeed);
	
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	user_main_printf("begin------------------------");
  while (1)
  {
		led_shan();
		
		
		
		
		
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
	LED2_ON;
	HAL_Delay(200);
	LED1_OFF;
	LED2_OFF;
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
