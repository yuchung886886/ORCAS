/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_cmds_handle.h"
#include "fire_ctrl_utils.h"
#include "orientation_ctrl_utils.h"
#include "tof_sensors_ctrl_utils.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
#define PERODIC_ROUTINES_DISPATCHED__FIRE_CTRL			0x01
#define PERODIC_ROUTINES_DISPATCHED__ORIENTATION_CTRL	0x02
#define PERODIC_ROUTINES_DISPATCHED__UART_CMDS_HANDLING	0x04
#define PERODIC_ROUTINES_DISPATCHED__TOF_SENSORS_CTRL	0x08
uint8_t perodic_routines_dispatched = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);
#ifdef TOF_SENSORS_CTRL
  tof_sensors_init();
#endif
#ifdef FIRE_CTRL
  fire_ctrl_init();
#endif
  orientation_ctrl_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(perodic_routines_dispatched & PERODIC_ROUTINES_DISPATCHED__FIRE_CTRL){
		perodic_routines_dispatched &= ~PERODIC_ROUTINES_DISPATCHED__FIRE_CTRL;
		fire_ctrl_perodic_routines();
	}
	if(perodic_routines_dispatched & PERODIC_ROUTINES_DISPATCHED__ORIENTATION_CTRL){
		perodic_routines_dispatched &= ~PERODIC_ROUTINES_DISPATCHED__ORIENTATION_CTRL;
		orientation_ctrl_perodic_routines();
	}
	if(perodic_routines_dispatched & PERODIC_ROUTINES_DISPATCHED__UART_CMDS_HANDLING){
		perodic_routines_dispatched &= ~PERODIC_ROUTINES_DISPATCHED__UART_CMDS_HANDLING;
		uart_cmds_handle_perodic_routines();
	}
	if(perodic_routines_dispatched & PERODIC_ROUTINES_DISPATCHED__TOF_SENSORS_CTRL){
		perodic_routines_dispatched &= ~PERODIC_ROUTINES_DISPATCHED__TOF_SENSORS_CTRL;
		tof_sensors_perodic_routines();
	}


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 250;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, FEEDER_MOTOR_DIR_Pin|FEEDER_MOTOR_SLEEP_Pin|RADAR_MOTOR_STEP_Pin|RADAR_MOTOR_DIR_Pin
                          |PAN_MOTOR_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RED_DOT_EN_Pin|TILT_MOTOR_STEP_Pin|TILT_MOTOR_DIR_Pin|FEEDER_MOTOR_STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RADAR_MOTOR_SLEEP_GPIO_Port, RADAR_MOTOR_SLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PAN_MOTOR_DIR_GPIO_Port, PAN_MOTOR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_D2_Pin|LED_D1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : FEEDER_MOTOR_DIR_Pin FEEDER_MOTOR_SLEEP_Pin RADAR_MOTOR_STEP_Pin RADAR_MOTOR_DIR_Pin
                           PAN_MOTOR_STEP_Pin */
  GPIO_InitStruct.Pin = FEEDER_MOTOR_DIR_Pin|FEEDER_MOTOR_SLEEP_Pin|RADAR_MOTOR_STEP_Pin|RADAR_MOTOR_DIR_Pin
                          |PAN_MOTOR_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_DOT_EN_Pin TILT_MOTOR_STEP_Pin TILT_MOTOR_DIR_Pin FEEDER_MOTOR_STEP_Pin
                           LED_D2_Pin LED_D1_Pin */
  GPIO_InitStruct.Pin = RED_DOT_EN_Pin|TILT_MOTOR_STEP_Pin|TILT_MOTOR_DIR_Pin|FEEDER_MOTOR_STEP_Pin
                          |LED_D2_Pin|LED_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PAN_MOTOR_ORIGIN_Pin */
  GPIO_InitStruct.Pin = PAN_MOTOR_ORIGIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PAN_MOTOR_ORIGIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : AEG_PISTON_ENDSTOP_Pin */
  GPIO_InitStruct.Pin = AEG_PISTON_ENDSTOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(AEG_PISTON_ENDSTOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADAR_MOTOR_SLEEP_Pin */
  GPIO_InitStruct.Pin = RADAR_MOTOR_SLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RADAR_MOTOR_SLEEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RADAR_MOTOR_ORIGIN_Pin */
  GPIO_InitStruct.Pin = RADAR_MOTOR_ORIGIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RADAR_MOTOR_ORIGIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PAN_MOTOR_DIR_Pin */
  GPIO_InitStruct.Pin = PAN_MOTOR_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PAN_MOTOR_DIR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
#define GLOBAL_TIMER_INT_PERIOD_US	250
uint16_t global_timer_count_us = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2){
		global_timer_count_us += GLOBAL_TIMER_INT_PERIOD_US;
		if(global_timer_count_us >= 10000){
			global_timer_count_us = 0;
		}

		if(global_timer_count_us % UART_CMDS_HANDLE_ROUTINES_MIN_PERIOD_US == 0){
			perodic_routines_dispatched |= PERODIC_ROUTINES_DISPATCHED__UART_CMDS_HANDLING;
		}

#ifdef FIRE_CTRL
		if(global_timer_count_us % FIRE_CTRL_ROUTINES_MIN_PERIOD_US == 0){
			perodic_routines_dispatched |= PERODIC_ROUTINES_DISPATCHED__FIRE_CTRL;
		}
#endif
		if(global_timer_count_us % ORIENTATION_CTRL_ROUTINES_MIN_PERIOD_US == 0){
			perodic_routines_dispatched |= PERODIC_ROUTINES_DISPATCHED__ORIENTATION_CTRL;
		}
#ifdef TOF_SENSORS_CTRL
		if(global_timer_count_us % TOF_SENSORS_ROUTINES_MIN_PERIOD_US == 0){
			perodic_routines_dispatched |= PERODIC_ROUTINES_DISPATCHED__TOF_SENSORS_CTRL;
		}
#endif
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
#ifdef FIRE_CTRL
	if(GPIO_Pin == AEG_PISTON_ENDSTOP_Pin){
		fire_ctrl_isr(GPIO_Pin);
	}
#endif
#ifdef ORIENTATION_CTRL
	if(GPIO_Pin == PAN_MOTOR_ORIGIN_Pin){
		orientation_ctrl_isr(GPIO_Pin);
	}
#endif
#ifdef TOF_SENSORS_CTRL
	if(GPIO_Pin == RADAR_MOTOR_ORIGIN_Pin){
		tof_sensors_isr(GPIO_Pin);
	}
#endif
}

void MX_I2C_ForceClearBusyFlag(I2C_HandleTypeDef *hi2c,
							   GPIO_TypeDef* sda_gpio_port, uint16_t sda_gpio_pin,
							   GPIO_TypeDef* scl_gpio_port, uint16_t scl_gpio_pin){
	GPIO_InitTypeDef GPIO_InitStructure;

	// 1. Disable the I2C peripheral by clearing the PE bit in I2Cx_CR1 register
	__HAL_I2C_DISABLE(hi2c);
	// HAL_I2C_DeInit(hi2c);
	// 2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level
	GPIO_InitStructure.Pin 			= sda_gpio_pin;
	GPIO_InitStructure.Mode         = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStructure.Pull         = GPIO_NOPULL;
	GPIO_InitStructure.Speed        = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(sda_gpio_port, &GPIO_InitStructure);
	HAL_GPIO_WritePin(sda_gpio_port, sda_gpio_pin, GPIO_PIN_SET);
	GPIO_InitStructure.Pin 			= scl_gpio_pin;
	GPIO_InitStructure.Mode         = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStructure.Pull         = GPIO_NOPULL;
	GPIO_InitStructure.Speed        = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(scl_gpio_port, &GPIO_InitStructure);
	HAL_GPIO_WritePin(scl_gpio_port, scl_gpio_pin, GPIO_PIN_SET);
	// 3. Check SCL and SDA High level
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(sda_gpio_port, sda_gpio_pin)){asm("nop");}
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(scl_gpio_port, scl_gpio_pin)){asm("nop");}
	// 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level
	HAL_GPIO_WritePin(sda_gpio_port, sda_gpio_pin, GPIO_PIN_RESET);
	// 5. Check SDA Low level.
	while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(sda_gpio_port, sda_gpio_pin)){asm("nop");}
	// 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level
	HAL_GPIO_WritePin(scl_gpio_port, scl_gpio_pin, GPIO_PIN_RESET);
	// 7. Check SCL Low level.
	while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(scl_gpio_port, scl_gpio_pin)){asm("nop");}
	// 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level
	HAL_GPIO_WritePin(scl_gpio_port, scl_gpio_pin, GPIO_PIN_SET);
	// 9. Check SCL High level.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(scl_gpio_port, scl_gpio_pin)){asm("nop");}
	// 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level
	HAL_GPIO_WritePin(sda_gpio_port, sda_gpio_pin, GPIO_PIN_SET);
	// 11. Check SDA High level.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(sda_gpio_port, sda_gpio_pin)){asm("nop");}
	// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain
	GPIO_InitStructure.Pin = sda_gpio_pin|scl_gpio_pin;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(sda_gpio_port, &GPIO_InitStructure);
	// 13. Set SWRST bit in I2Cx_CR1 register
    SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    // 14. Clear SWRST bit in I2Cx_CR1 register
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    // 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
    // __HAL_I2C_ENABLE(hi2c);
    HAL_I2C_Init(hi2c);
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
