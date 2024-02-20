/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Set_Pin GPIO_PIN_4
#define Set_GPIO_Port GPIOE
#define Set_EXTI_IRQn EXTI4_IRQn
#define Speed_Increase_Pin GPIO_PIN_5
#define Speed_Increase_GPIO_Port GPIOE
#define Speed_Increase_EXTI_IRQn EXTI9_5_IRQn
#define Speed_Decrease_Pin GPIO_PIN_6
#define Speed_Decrease_GPIO_Port GPIOE
#define Speed_Decrease_EXTI_IRQn EXTI9_5_IRQn
#define Gap_Increase_Pin GPIO_PIN_13
#define Gap_Increase_GPIO_Port GPIOC
#define Gap_Increase_EXTI_IRQn EXTI15_10_IRQn
#define Gap_Decrease_Pin GPIO_PIN_0
#define Gap_Decrease_GPIO_Port GPIOC
#define Gap_Decrease_EXTI_IRQn EXTI0_IRQn
#define AdaptiveCruiseControl_Pin GPIO_PIN_1
#define AdaptiveCruiseControl_GPIO_Port GPIOC
#define AdaptiveCruiseControl_EXTI_IRQn EXTI1_IRQn
#define Engine_Status_Pin GPIO_PIN_2
#define Engine_Status_GPIO_Port GPIOC
#define Engine_Status_EXTI_IRQn EXTI2_IRQn
#define CruiseControl_Pin GPIO_PIN_3
#define CruiseControl_GPIO_Port GPIOC
#define CruiseControl_EXTI_IRQn EXTI3_IRQn
#define ENGINE_CONTROL_Pin GPIO_PIN_7
#define ENGINE_CONTROL_GPIO_Port GPIOA
#define KY_SIGNAL_Pin GPIO_PIN_1
#define KY_SIGNAL_GPIO_Port GPIOB
#define LEDSTAT_Pin GPIO_PIN_7
#define LEDSTAT_GPIO_Port GPIOE
#define TRIGGER_Pin GPIO_PIN_8
#define TRIGGER_GPIO_Port GPIOE
#define ECHO_Pin GPIO_PIN_9
#define ECHO_GPIO_Port GPIOE
#define StopControl_Pin GPIO_PIN_10
#define StopControl_GPIO_Port GPIOE
#define StopControl_EXTI_IRQn EXTI15_10_IRQn
#define StopControl2_Pin GPIO_PIN_12
#define StopControl2_GPIO_Port GPIOE
#define StopControl2_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
