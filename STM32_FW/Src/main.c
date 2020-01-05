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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU_ADDR 0xD0
#define TARGET_PERIOD 5
#define PI 3.14159265
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */
int16_t old_angle = 0;
int16_t pos_dead_band = 0;
int16_t neg_dead_band = -0;
float i_error = 0;
int dead = 0;
int16_t current_speed = 0;

// Configurable parameters
float ki = 2;
float kp = 15;
float kd = 1;
int do_usb = 0; // Set to 1 to do USB transactions
int include_delay = 0; // Set to 1 to include a wait in the loop to achieve target loop time
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef MPU_GetReg(uint8_t, uint8_t*, uint8_t);
HAL_StatusTypeDef MPU_SetReg(uint8_t, uint8_t);
uint8_t MPU_Exists();
void SET_PWM_SPEED(int32_t);
int8_t sign(int16_t x);
int16_t balance(float, float, uint32_t);

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
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM16_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  HAL_Delay(100);
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  HAL_Delay(700);

  while(!MPU_Exists())
  {
	  if(do_usb) CDC_Transmit_FS((uint8_t*)"MPU did not response\n\r", 22);
  }

  if(do_usb) CDC_Transmit_FS((uint8_t*)"MPU is alive\n\r", 14);

  MPU_SetReg(0x6b, 0x00); //Set power mode
  MPU_SetReg(0x1c, 0x10); // Set accelerometers to +/-4g
  MPU_SetReg(0x1b, 0x10); //Set gyros to +/-500deg/sec

  HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_ALL);
  HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_ALL);
  HAL_TIMEx_PWMN_Start(&htim14, TIM_CHANNEL_1);

  uint16_t delay_time = TARGET_PERIOD;
  uint8_t acc_regs[6];
  float acc_vals[3];
  uint8_t gyro_regs[6];
  float gyro_vals[3];
  char usb_buf[150];
  uint8_t usb_buf_len;
  uint8_t usb_error_flag = 0;
  float angle = 0;
  uint32_t t = HAL_GetTick();

  while (1)
  {
	  // Read Acceleration values
	  MPU_GetReg(0x3b, acc_regs, 6);
	  acc_vals[0] = ((int16_t)(acc_regs[0] << 8) + acc_regs[1])/8192.0; //Convert force to grams
	  acc_vals[1] = ((int16_t)(acc_regs[2] << 8) + acc_regs[3])/8192.0; //Convert force to grams
	  acc_vals[2] = ((int16_t)(acc_regs[4] << 8) + acc_regs[5])/8192.0; //Convert force to grams

	  // Check for accelerator saturation
	  if((acc_vals[0] > 3.9) | (acc_vals[1] > 3.9) | (acc_vals[2] > 3.9) | (acc_vals[0] < -3.9) | (acc_vals[1] < -3.9) | (acc_vals[2] < -3.9))
	  {
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  }

	  // Read Gyro values and convert to
	  MPU_GetReg(0x43, gyro_regs, 6);
	  gyro_vals[0] = ((int16_t)(gyro_regs[0] << 8) + gyro_regs[1])/65.5 + 0.1620; //Convert to degs/s
	  gyro_vals[1] = ((int16_t)(gyro_regs[2] << 8) + gyro_regs[3])/65.5; //Convert to degs/s
	  gyro_vals[2] = ((int16_t)(gyro_regs[4] << 8) + gyro_regs[5])/65.5 - 1.1653; //Convert to degs/s

	  // Check for gyro saturation
	  if((gyro_vals[0] > 450) | (gyro_vals[1] > 450) | (gyro_vals[2] > 450) | (gyro_vals[0] < -450) | (gyro_vals[1] < -450) | (gyro_vals[2] < -450))
	  {
		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	  }

	  // Calculate the accelerometer angle
	  float accAngle = (atan2(acc_vals[0],acc_vals[1]) * 180 / PI);

	  // Get time delta
	  uint32_t t_now = HAL_GetTick();
	  uint32_t t_delta = t_now - t;
	  delay_time += (TARGET_PERIOD - t_delta);
	  t = t_now;

	  // Combine accelerometer and gyro angles
	  angle = 0.98*(angle + gyro_vals[2]*t_delta/1000) + 0.02*accAngle;

	  // Send USB
	  if(do_usb)
	  {
		  usb_buf_len = 0;
		  usb_buf_len = sprintf(usb_buf, "Dt: %lu\tAccx: %d\tAccy: %d\tAccz: %d\tAccAngle: %d\n\r", t_delta, (int16_t)(acc_vals[0]*1000),(int16_t)(acc_vals[1]*1000),(int16_t)(acc_vals[2]*1000),(int16_t)(accAngle));
		  HAL_StatusTypeDef usb_result = CDC_Transmit_FS((uint8_t*)usb_buf, usb_buf_len);
		  usb_error_flag = (usb_result == HAL_OK ? 0 : 1);
	  }

	  int16_t speed = balance(angle+1, gyro_vals[2], t_delta);

	  if(include_delay)
	  {
		  if(delay_time > 0) HAL_Delay(delay_time);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 100;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.RepetitionCounter = 0;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MPU_CLK_GPIO_Port, MPU_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MPU_FSYNC_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MPU_INT_Pin */
  GPIO_InitStruct.Pin = MPU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU_CLK_Pin */
  GPIO_InitStruct.Pin = MPU_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MPU_CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MPU_FSYNC_Pin LED_Pin */
  GPIO_InitStruct.Pin = MPU_FSYNC_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int8_t sign(int16_t x)
{
    return (x > 0) - (x < 0);
}

void SET_PWM_SPEED(int32_t value)
{
    int16_t m2 = (value > 0 ? value : 0);
    int16_t m1 = (value < 0 ? -1*value : 0);

    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, m1);
    __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, m2);
    HAL_TIMEx_PWMN_Start(&htim16, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
}

int16_t balance(float angle, float delta_angle, uint32_t t_delta)
{
	if(abs(angle) > 30 || dead > 0){
		SET_PWM_SPEED(0);
		i_error = 0;
		dead = 1;
		return 0;
	}

	old_angle = angle;
	i_error += angle * t_delta / 1000;
	int16_t desired_speed = (int16_t)(i_error*ki + angle*kp + delta_angle*kd);
	desired_speed += (desired_speed > 0 ? pos_dead_band : (desired_speed < 0 ? neg_dead_band : 0));

	current_speed = desired_speed;

	SET_PWM_SPEED(current_speed);

	return desired_speed;
}

uint8_t MPU_Exists()
{
  uint8_t response;
  HAL_StatusTypeDef status = MPU_GetReg(0x75, &response, 1);
  if(status == HAL_OK && response == 0x68)
  {
	  return 1;
  }
  else
  {
	  return 0;
  }
}

HAL_StatusTypeDef MPU_GetReg(uint8_t reg, uint8_t* response, uint8_t len)
{
	return HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, (uint16_t) reg, I2C_MEMADD_SIZE_8BIT, response, len, 100);
}

HAL_StatusTypeDef MPU_SetReg(uint8_t reg, uint8_t value)
{
	return HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, (uint16_t) reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
