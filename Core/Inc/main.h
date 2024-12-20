/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f2xx_hal.h"

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
#define DO_vent_Pin GPIO_PIN_13
#define DO_vent_GPIO_Port GPIOC
#define DO_unload_Pin GPIO_PIN_14
#define DO_unload_GPIO_Port GPIOC
#define DO_fb_fc_Pin GPIO_PIN_15
#define DO_fb_fc_GPIO_Port GPIOC
#define DO_contactor_Pin GPIO_PIN_0
#define DO_contactor_GPIO_Port GPIOC
#define fb_contactor_Pin GPIO_PIN_1
#define fb_contactor_GPIO_Port GPIOC
#define TD_DM_Pin GPIO_PIN_2
#define TD_DM_GPIO_Port GPIOC
#define start_Pin GPIO_PIN_3
#define start_GPIO_Port GPIOC
#define reset_trigger_Pin GPIO_PIN_2
#define reset_trigger_GPIO_Port GPIOB
#define er_total_Pin GPIO_PIN_11
#define er_total_GPIO_Port GPIOB
#define led_green_Pin GPIO_PIN_3
#define led_green_GPIO_Port GPIOB
#define type_comp_Pin GPIO_PIN_4
#define type_comp_GPIO_Port GPIOB
#define sa2_Pin GPIO_PIN_5
#define sa2_GPIO_Port GPIOB
#define sa3_Pin GPIO_PIN_6
#define sa3_GPIO_Port GPIOB
#define led_can_Pin GPIO_PIN_7
#define led_can_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
