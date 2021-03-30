/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void led_shan(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SG1_Pin GPIO_PIN_2
#define SG1_GPIO_Port GPIOE
#define SG2_Pin GPIO_PIN_3
#define SG2_GPIO_Port GPIOE
#define SG3_Pin GPIO_PIN_4
#define SG3_GPIO_Port GPIOE
#define SG4_Pin GPIO_PIN_5
#define SG4_GPIO_Port GPIOE
#define SG5_Pin GPIO_PIN_6
#define SG5_GPIO_Port GPIOE
#define SG6_Pin GPIO_PIN_13
#define SG6_GPIO_Port GPIOC
#define SG7_Pin GPIO_PIN_14
#define SG7_GPIO_Port GPIOC
#define SG8_Pin GPIO_PIN_15
#define SG8_GPIO_Port GPIOC
#define SG13_Pin GPIO_PIN_0
#define SG13_GPIO_Port GPIOC
#define SG14_Pin GPIO_PIN_1
#define SG14_GPIO_Port GPIOC
#define SG15_Pin GPIO_PIN_2
#define SG15_GPIO_Port GPIOC
#define SG16_Pin GPIO_PIN_3
#define SG16_GPIO_Port GPIOC
#define MAX7219_CS_Pin GPIO_PIN_6
#define MAX7219_CS_GPIO_Port GPIOA
#define SJ1_Pin GPIO_PIN_7
#define SJ1_GPIO_Port GPIOE
#define SJ2_Pin GPIO_PIN_8
#define SJ2_GPIO_Port GPIOE
#define PWM11_Pin GPIO_PIN_9
#define PWM11_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOE
#define PWM12_Pin GPIO_PIN_11
#define PWM12_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_12
#define LED2_GPIO_Port GPIOE
#define PWM21_Pin GPIO_PIN_13
#define PWM21_GPIO_Port GPIOE
#define SJ3_Pin GPIO_PIN_14
#define SJ3_GPIO_Port GPIOE
#define SJ4_Pin GPIO_PIN_15
#define SJ4_GPIO_Port GPIOE
#define SJ5_Pin GPIO_PIN_10
#define SJ5_GPIO_Port GPIOB
#define SJ6_Pin GPIO_PIN_11
#define SJ6_GPIO_Port GPIOB
#define PWM31_Pin GPIO_PIN_6
#define PWM31_GPIO_Port GPIOC
#define PWM32_Pin GPIO_PIN_7
#define PWM32_GPIO_Port GPIOC
#define PWM41_Pin GPIO_PIN_8
#define PWM41_GPIO_Port GPIOC
#define PWM42_Pin GPIO_PIN_9
#define PWM42_GPIO_Port GPIOC
#define PWM22_Pin GPIO_PIN_11
#define PWM22_GPIO_Port GPIOA
#define LED7Seg_CLK_Pin GPIO_PIN_10
#define LED7Seg_CLK_GPIO_Port GPIOC
#define LED7Seg_CS_Pin GPIO_PIN_11
#define LED7Seg_CS_GPIO_Port GPIOC
#define LED7Seg_DIN_Pin GPIO_PIN_0
#define LED7Seg_DIN_GPIO_Port GPIOD
#define SW2_Pin GPIO_PIN_4
#define SW2_GPIO_Port GPIOD
#define SW1_Pin GPIO_PIN_7
#define SW1_GPIO_Port GPIOD
#define SG9_Pin GPIO_PIN_8
#define SG9_GPIO_Port GPIOB
#define SG10_Pin GPIO_PIN_9
#define SG10_GPIO_Port GPIOB
#define SG11_Pin GPIO_PIN_0
#define SG11_GPIO_Port GPIOE
#define SG12_Pin GPIO_PIN_1
#define SG12_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define CTR1_Pin GPIO_PIN_0
#define CTR1_GPIO_Port GPIOD

#define SW1 HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin)
#define SW2 HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin)
#define LED1_ON HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET)
#define LED2_ON HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET)
#define LED1_OFF HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET)
#define LED2_OFF HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET)




#define PRESSED 0
#define UNPRESSED 1

extern int32_t y_speed;
extern int32_t x_speed;
extern int32_t a_speed;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
