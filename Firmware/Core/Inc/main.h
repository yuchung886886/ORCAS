/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define FW_VER	"ORCAS_20250801 "
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void MX_I2C_ForceClearBusyFlag(I2C_HandleTypeDef *hi2c,
							   GPIO_TypeDef* sda_gpio_port, uint16_t sda_gpio_pin,
							   GPIO_TypeDef* scl_gpio_port, uint16_t scl_gpio_pin);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FEEDER_MOTOR_DIR_Pin GPIO_PIN_13
#define FEEDER_MOTOR_DIR_GPIO_Port GPIOC
#define AEG_MOTOR_PWM_Pin GPIO_PIN_6
#define AEG_MOTOR_PWM_GPIO_Port GPIOA
#define TRACER_LED_PWM_Pin GPIO_PIN_7
#define TRACER_LED_PWM_GPIO_Port GPIOA
#define RED_DOT_EN_Pin GPIO_PIN_0
#define RED_DOT_EN_GPIO_Port GPIOB
#define TARGETING_LED_PWM_Pin GPIO_PIN_1
#define TARGETING_LED_PWM_GPIO_Port GPIOB
#define PAN_MOTOR_ORIGIN_Pin GPIO_PIN_6
#define PAN_MOTOR_ORIGIN_GPIO_Port GPIOC
#define PAN_MOTOR_ORIGIN_EXTI_IRQn EXTI9_5_IRQn
#define AEG_PISTON_ENDSTOP_Pin GPIO_PIN_7
#define AEG_PISTON_ENDSTOP_GPIO_Port GPIOC
#define AEG_PISTON_ENDSTOP_EXTI_IRQn EXTI9_5_IRQn
#define FEEDER_MOTOR_SLEEP_Pin GPIO_PIN_8
#define FEEDER_MOTOR_SLEEP_GPIO_Port GPIOC
#define RADAR_MOTOR_SLEEP_Pin GPIO_PIN_8
#define RADAR_MOTOR_SLEEP_GPIO_Port GPIOA
#define RADAR_MOTOR_ORIGIN_Pin GPIO_PIN_15
#define RADAR_MOTOR_ORIGIN_GPIO_Port GPIOA
#define RADAR_MOTOR_ORIGIN_EXTI_IRQn EXTI15_10_IRQn
#define RADAR_MOTOR_STEP_Pin GPIO_PIN_10
#define RADAR_MOTOR_STEP_GPIO_Port GPIOC
#define RADAR_MOTOR_DIR_Pin GPIO_PIN_11
#define RADAR_MOTOR_DIR_GPIO_Port GPIOC
#define PAN_MOTOR_STEP_Pin GPIO_PIN_12
#define PAN_MOTOR_STEP_GPIO_Port GPIOC
#define PAN_MOTOR_DIR_Pin GPIO_PIN_2
#define PAN_MOTOR_DIR_GPIO_Port GPIOD
#define TILT_MOTOR_STEP_Pin GPIO_PIN_3
#define TILT_MOTOR_STEP_GPIO_Port GPIOB
#define TILT_MOTOR_DIR_Pin GPIO_PIN_4
#define TILT_MOTOR_DIR_GPIO_Port GPIOB
#define FEEDER_MOTOR_STEP_Pin GPIO_PIN_5
#define FEEDER_MOTOR_STEP_GPIO_Port GPIOB
#define LED_D2_Pin GPIO_PIN_8
#define LED_D2_GPIO_Port GPIOB
#define LED_D1_Pin GPIO_PIN_9
#define LED_D1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
