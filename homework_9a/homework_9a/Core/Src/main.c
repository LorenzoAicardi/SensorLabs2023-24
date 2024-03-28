/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct nota {
	int period;
	int pulse;
	int eights;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRESCALER_TIM1 100-1
#define PRESCALER_TIM2 84000-1

#define TIME_EIGHTS 100.0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define DIM_ROWS 4
#define DIM_COLS 4
#define DIM_STRING 100

#define PE_DO4 3204
#define PE_DOD4 3032
#define PE_RE4 2857
#define PE_RED4 2700
#define PE_MI4 2545
#define PE_FA4 2406
#define PE_FAD4 2270
#define PE_SOL4 2142
#define PE_SOLD4 2024
#define PE_LA4 1909
#define PE_LAD4 1802
#define PE_SI4 1700
// pulses
#define PU_DO4 1603
#define PU_DOD4 1516
#define PU_RE4 1428
#define PU_RED4 1350
#define PU_MI4 1272
#define PU_FA4 1203
#define PU_FAD4 1135
#define PU_SOL4 1071
#define PU_SOLD4 1012
#define PU_LA4 954
#define PU_LAD4 901
#define PU_SI4 850

struct nota canzone[] = {
		{PE_SOL4,PU_SOL4,3}, {PE_LA4,PU_LA4, 1},
		{PE_SOL4,PU_SOL4,2}, {PE_FA4,PU_FA4, 2},

		{PE_MI4,PU_MI4, 2},  {PE_FA4,PU_FA4, 2},
		{PE_SOL4,PU_SOL4,4},

		{PE_RE4,PU_RE4,2}, {PE_MI4,PU_MI4,2},
		{PE_FA4,PU_FA4,4},

		{PE_MI4,PU_MI4,2}, {PE_FA4,PU_FA4,2},
		{PE_SOL4,PU_SOL4,4},

		{PE_SOL4,PU_SOL4,3}, {PE_LA4,PU_LA4, 1},
		{PE_SOL4,PU_SOL4,2}, {PE_FA4,PU_FA4, 2},

		{PE_MI4,PU_MI4, 2},  {PE_FA4,PU_FA4, 2},
		{PE_SOL4,PU_SOL4,4},

		{PE_RE4,PU_RE4,4}, {PE_SOL4,PU_SOL4,4},

		{PE_MI4,PU_MI4, 2},{PE_DO4,PU_DO4, 6},
};
// 5 V is the voltage at which the pull-up network brings the value of the
// row
#define DEBOUNCING_THRESHOLD 5
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// relative to GPIO_C
int rows[DIM_ROWS] = {GPIO_PIN_12, GPIO_PIN_13,GPIO_PIN_2, GPIO_PIN_3, }; // to read from
// relative to GPIO_C
int cols[DIM_ROWS] = {GPIO_PIN_8,GPIO_PIN_9 , GPIO_PIN_10, GPIO_PIN_11}; // to write on

// out buffer
char str_buffer[DIM_STRING];
int str_buffer_len=0;

// History = 0: button not currently pressed since when I notice not pressed I set zero
// History > 1: button currently pressed since each time I press it I increment the value
uint16_t history[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
int buttonValueFromRowColumn(int row, int column);
void scanButtons();
void printButtonNumber(int number);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void scanButtons(){
	uint16_t buttonState;
	for(int c=0; c<DIM_COLS; c++) {
		// Enable the column row
		HAL_GPIO_WritePin(GPIOC, cols[c], GPIO_PIN_SET);
		HAL_Delay(5);

		for(int r=0; r<DIM_ROWS; r++){
			// Read the row
			buttonState = HAL_GPIO_ReadPin(GPIOC, rows[r]);

			// Update history of each button (if I read a zero)
			if(buttonState==GPIO_PIN_RESET){
				history[buttonValueFromRowColumn(r, c)]++;
			} else {
				history[buttonValueFromRowColumn(r, c)] = 0;
			}

			// If button pressing history reached the DEBOUNCING_THRESHOLD, print its value
			if(history[buttonValueFromRowColumn(r, c)]==DEBOUNCING_THRESHOLD) {
				printButtonNumber(buttonValueFromRowColumn(r, c));
			}

			if(history[0] == DEBOUNCING_THRESHOLD){
				// -- SETUP NOTE --
				// Set AutoReload Register to the note period
				__HAL_TIM_SET_AUTORELOAD(&htim1,canzone[10].period);
				// Set CCR2 to the note pulse
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, canzone[10].pulse);
				// Play the note: start PWM timer 1 on channel 2
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			} else if (history[3] == DEBOUNCING_THRESHOLD) {
				// -- SETUP NOTE --
				// Set AutoReload Register to the note period
				__HAL_TIM_SET_AUTORELOAD(&htim1,canzone[1].period);
				// Set CCR2 to the note pulse
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, canzone[1].pulse);
				// Play the note: start PWM timer 1 on channel 2
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			} else if (history[12] == DEBOUNCING_THRESHOLD) {
				// -- SETUP NOTE --
				// Set AutoReload Register to the note period
				__HAL_TIM_SET_AUTORELOAD(&htim1,canzone[2].period);
				// Set CCR2 to the note pulse
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, canzone[2].pulse);
				// Play the note: start PWM timer 1 on channel 2
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			} else if (history[15] == DEBOUNCING_THRESHOLD) {
				// -- SETUP NOTE --
				// Set AutoReload Register to the note period
				__HAL_TIM_SET_AUTORELOAD(&htim1,canzone[3].period);
				// Set CCR2 to the note pulse
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, canzone[3].pulse);
				// Play the note: start PWM timer 1 on channel 2
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
			} else if(history[buttonValueFromRowColumn(r, c)]==DEBOUNCING_THRESHOLD){
				if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_RESET)
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
				else
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
			}
		}

		// Disable the column row
		HAL_GPIO_WritePin(GPIOC, cols[c], GPIO_PIN_RESET);
	}
}

int buttonValueFromRowColumn(int row, int column) {
	// we want to number the buttons from the top row to the bottom one
	// and from the left column to the right one
	// (e.g. row 1, column 1 is equal to 1, then the one on the right
	// is the number 2 and so on).
	//
	// So, in order to convert row-column to the value we compute:
	// row*number_of_columns+column.
	return row*DIM_COLS+column;
}

void printButtonNumber(int number){
	// to print from 0 to F I print 0-15 in hex
	str_buffer_len = snprintf(str_buffer, DIM_STRING, "Button %X pressed\n\r", number);
	HAL_UART_Transmit(&huart2, str_buffer, str_buffer_len, 50);
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
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_SET_PRESCALER(&htim1,PRESCALER_TIM1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  scanButtons();
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
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
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC2 PC3 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
