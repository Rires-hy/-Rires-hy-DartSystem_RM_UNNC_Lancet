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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define POWER1_CTRL_Pin GPIO_PIN_2
#define POWER1_CTRL_GPIO_Port GPIOH
#define POWER2_CTRL_Pin GPIO_PIN_3
#define POWER2_CTRL_GPIO_Port GPIOH
#define POWER3_CTRL_Pin GPIO_PIN_4
#define POWER3_CTRL_GPIO_Port GPIOH
#define POWER4_CTRL_Pin GPIO_PIN_5
#define POWER4_CTRL_GPIO_Port GPIOH
#define KEY_Pin GPIO_PIN_2
#define KEY_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_11
#define LED_RED_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOF
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim1;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

extern osThreadId defaultTaskHandle;
extern osThreadId myTask02Handle;

#define MODE_1_READY_FEED 1
#define MODE_2_FEED_DART 2
#define MODE_3_RELEASE 3

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SWITCH_Pin GPIO_PIN_13
#define SWITCH_GPIO_Port GPIOG
#define SWITCH_EXTI_IRQn EXTI15_10_IRQn
#define switch_limit_Pin GPIO_PIN_6
#define switch_limit_GPIO_Port GPIOE
#define IN1_Pin GPIO_PIN_9
#define IN1_GPIO_Port GPIOI
#define IN2_Pin GPIO_PIN_10
#define IN2_GPIO_Port GPIOF
#define limit1_Pin GPIO_PIN_2
#define limit1_GPIO_Port GPIOC
#define limit1_EXTI_IRQn EXTI2_IRQn
#define TNT_Pin GPIO_PIN_12
#define TNT_GPIO_Port GPIOD
#define ENA_Pin GPIO_PIN_5
#define ENA_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_11
#define LED_R_GPIO_Port GPIOE
#define limit2_Pin GPIO_PIN_0
#define limit2_GPIO_Port GPIOB
#define limit2_EXTI_IRQn EXTI0_IRQn
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOF
/* USER CODE BEGIN Private defines */

typedef struct
{
	uint8_t isSpeedCtrl;
	uint8_t isAngleCtrl;
	int Lap;
	int LastAngle;
	int CurrentAngle;
	int LastSpeed;
	int CurrentSpeed;
	int TargetAngle;
	int TargetLap;
	int TargetSpeed;
	int errorAngle;
	int errorSpeed;
	int errorTotal;
	float kPAngle;
	float kPSpeed;
} MotorControlInfo;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
