/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
TIM_HandleTypeDef htim1;
I2C_HandleTypeDef hi2c1;

#define START_HOUR_1   8   // Start hour for interval 1
#define START_MINUTE_1 0   // Start minute for interval 1
#define DURATION_1     10  // Duration in minutes for interval 1

#define START_HOUR_2   16  // Start hour for interval 2
#define START_MINUTE_2 31  // Start minute for interval 2
#define DURATION_2     2   // Duration in minutes for interval 2


// Reads the registers of the of the connected RTC device
// Connected on I2C1 SDA and SCL lines
uint8_t ReadRTCRegister(uint8_t register_address)
{
    uint8_t data;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, 0x68 << 1, register_address, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        printf("Error reading register 0x%02X\n", register_address);
        return 0xFF; // Return error code
    }
    return data;
}


// Reads and returns the seconds, minutes and, hours of the RTC
void ReadRTC(uint8_t *hours, uint8_t *minutes, uint8_t *seconds)
{
    // Read RTC registers
    uint8_t reg_seconds = ReadRTCRegister(0x00);
    uint8_t reg_minutes = ReadRTCRegister(0x01);
    uint8_t reg_hours = ReadRTCRegister(0x02);

    // Convert BCD to decimal
    *seconds = ((reg_seconds >> 4) * 10) + (reg_seconds & 0x0F);
    *minutes = ((reg_minutes >> 4) * 10) + (reg_minutes & 0x0F);
    *hours = ((reg_hours >> 4) * 10) + (reg_hours & 0x0F);
}

// Configurations of the PWM pins for the connected MOSFET
// Connected on TIM1_CH1
void ConfigureTIM1ForPWM(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    // Configure TIM1 for PWM mode
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = __HAL_TIM_GET_AUTORELOAD(&htim1); // Full duty cycle (match auto-reload value)
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    // Initialize TIM1 with PWM settings
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

}

// Function to check if current time matches the trigger times
static uint8_t active_interval = 0; // 0: none, 1: interval 1, 2: interval 2
static uint8_t timer_active = 0;

// Checks the time and triggers the TIM1_CH1 PWM line to trigger the stepper pump
void CheckAndTriggerTimer(void)
{
    uint8_t hours, minutes, seconds;
    ReadRTC(&hours, &minutes, &seconds);

    // Determine if the current time matches any of the start times
    uint8_t is_start_time_1 = (hours == START_HOUR_1 && minutes == START_MINUTE_1);
    uint8_t is_start_time_2 = (hours == START_HOUR_2 && minutes == START_MINUTE_2);

    if ((is_start_time_1 || is_start_time_2) && !timer_active)
    {
        uint8_t start_hours = hours;
        uint8_t start_minutes = minutes;
        uint8_t start_seconds = seconds;

        ConfigureTIM1ForPWM();
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Start PWM

        timer_active = 1;

        if (is_start_time_1) {
            active_interval = 1;
        } else if (is_start_time_2) {
            active_interval = 2;
        }

        while (1)
        {
            ReadRTC(&hours, &minutes, &seconds);

            // Calculate the total elapsed minutes
            int elapsed_minutes = (hours - start_hours) * 60 + (minutes - start_minutes);

            // Check if the duration for the active interval has passed
            if ((active_interval == 1 && elapsed_minutes >= DURATION_1) ||
                (active_interval == 2 && elapsed_minutes >= DURATION_2))
            {
                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); // Ensure PWM is stopped
                HAL_Delay(1000); // Adjust delay as necessary
                timer_active = 0;
                active_interval = 0; // Reset active interval
                break; // Exit the loop
            }

            HAL_Delay(1000); // Adjust delay as necessary
        }
    }
}


// Power up function to prime the system at power failure
void PowerUp(void)
{
    uint8_t start_hours, start_minutes, start_seconds;
    ReadRTC(&start_hours, &start_minutes, &start_seconds);

    uint8_t current_hours, current_minutes, current_seconds;

    // Configure TIM1 for PWM
    ConfigureTIM1ForPWM();
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // Start PWM

    // Poll until 10 minutes have passed
    while (1)
    {
        ReadRTC(&current_hours, &current_minutes, &current_seconds);

        // Calculate the total elapsed minutes
        int elapsed_minutes = (current_hours - start_hours) * 60 + (current_minutes - start_minutes);

        // Check if 10 minutes have passed
        if (elapsed_minutes >= 1)
        {
            // Stop PWM
        	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); // Ensure PWM is stopped
            HAL_Delay(1000); // Adjust delay as necessary
            break; // Exit the loop to proceed to the main while loop
        }

        // Add a short delay to avoid excessive polling
        HAL_Delay(1000); // Adjust delay as necessary
    }
}



#define RTC_ADDRESS 0x68 // I2C address of the RTC module

// Write's a single byte to a register
void WriteRTCRegister(uint8_t register_address, uint8_t data)
{
    HAL_I2C_Mem_Write(&hi2c1, RTC_ADDRESS << 1, register_address, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

// Set the date and time of the connected RTC
void SetRTCDateTime(uint8_t second, uint8_t minute, uint8_t hour, uint8_t day, uint8_t date, uint8_t month, uint8_t year)
{
    // Convert decimal to BCD (Binary-Coded Decimal)
    uint8_t data[7];
    data[0] = (second / 10) << 4 | (second % 10);
    data[1] = (minute / 10) << 4 | (minute % 10);
    data[2] = (hour / 10) << 4 | (hour % 10);
    data[3] = (day / 10) << 4 | (day % 10);
    data[4] = (date / 10) << 4 | (date % 10);
    data[5] = (month / 10) << 4 | (month % 10);
    data[6] = (year / 10) << 4 | (year % 10);

    // Write to RTC registers (order might vary based on RTC model)
    WriteRTCRegister(0x00, data[0]); // Seconds
    WriteRTCRegister(0x01, data[1]); // Minutes
    WriteRTCRegister(0x02, data[2]); // Hours
    WriteRTCRegister(0x03, data[3]); // Day
    WriteRTCRegister(0x04, data[4]); // Date
    WriteRTCRegister(0x05, data[5]); // Month
    WriteRTCRegister(0x06, data[6]); // Year
}
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  PowerUp();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
      CheckAndTriggerTimer();
      HAL_Delay(1000); // Delay to prevent excessive RTC polling
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
