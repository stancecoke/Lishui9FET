/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx_ll_tim.h"

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
void kingmeter_update(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BKL_Brake_Pin GPIO_PIN_13
#define BKL_Brake_GPIO_Port GPIOC
#define AI0_notknown_Pin GPIO_PIN_0
#define AI0_notknown_GPIO_Port GPIOA
#define SP_Throttle_Pin GPIO_PIN_4
#define SP_Throttle_GPIO_Port GPIOA
#define Battery_Voltage_Pin GPIO_PIN_6
#define Battery_Voltage_GPIO_Port GPIOA
#define Battery_Current_Pin GPIO_PIN_0
#define Battery_Current_GPIO_Port GPIOB
#define SC_Hall_3_Pin GPIO_PIN_10
#define SC_Hall_3_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define SA_Hall_1_Pin GPIO_PIN_15
#define SA_Hall_1_GPIO_Port GPIOA
#define SB_Hall_2_Pin GPIO_PIN_3
#define SB_Hall_2_GPIO_Port GPIOB
#define TA_PAS_Pin GPIO_PIN_4
#define TA_PAS_GPIO_Port GPIOB
#define TA_PAS_EXTI_IRQn EXTI4_15_IRQn
#define SS_Speed_Pin GPIO_PIN_9
#define SS_Speed_GPIO_Port GPIOB
#define SS_Speed_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */
typedef struct
{
	int16_t       	gain_p;
	int16_t       	gain_i;
	int16_t       	limit_i;
	int16_t       	limit_output;
	int16_t       	recent_value;
	int32_t       	setpoint;
	int32_t       	integral_part;
	int16_t       	max_step;
	int32_t       	out;
	int8_t       	shift;

}PI_control_t;

extern void UART_IdleItCallback(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
