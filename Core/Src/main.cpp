/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.cpp
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/**
 * Wichtige Informationen für das Verständnis
 * 	>	Aufgrund der UART-Ansteuerung werden die Treiber X und Z benutzt.
 * 		Es werden trotzdem X-Y-Koordinaten verwendet.
 * 	>
 */

/**
 * TODO Funktionen Kommentieren und Dokumentation anfügen
 * TODO Definition weiterer Konstanten und Makros für Pin-Zustände und Verzögerungen, um den Code lesbarer und wartbarer zu machen.
 * TODO Aufteilung der Hauptschleife in mehrere modulare Funktionen zur Bewältigung verschiedener Aufgaben
 * TODO Motoransteuerung Z
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "move.h"
#include "homing.h"
#include "driver.h"
#include "TMC2209.h"
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

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart8;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart8_tx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Test-Variablen */
/* Buffer */
uint8_t rxDatagram[32];

/* Sensorvariablen */
volatile uint8_t BatteryAlarm = false;

/* Positionsvariablen */
const uint16_t HomeX = 0;
const uint16_t HomeY = 0;
uint16_t HomePos[2] = { HomeX, HomeY };
volatile float PosX = 0; //in mm
volatile float PosY = 0; //in mm

/* PWM Ansteuerung der STEP-Pins durch Interrupt */
volatile int32_t PWMStepX = 0; // Zähler für die X PWM-Impulse
volatile uint16_t PWMCounterX = 0; // Zähler für die PWM-Impuls-Position
volatile uint32_t TargetStepsX = 0;

volatile uint16_t PWMPeriod = 1000; // Periode des PWM-Signals in µs
volatile uint16_t PWMPulseWidth = 500; // Breite des PWM-Impulses (Duty Cycle)
volatile uint8_t PWMEnabledX = false; // Variable zum Ein-/Ausschalten der X-PWM
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
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

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

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
	MX_DMA_Init();
	MX_UART8_Init();
	MX_USART2_UART_Init();
	MX_CRC_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	/* Peripheral Configuration */
	TMC2209 tmc2209X, tmc2209Z;
	/* CLK Configuration */
	HAL_TIM_Base_Start_IT(&htim2);

	/* GPIO Configuration */
	HAL_GPIO_WritePin(X_EN_GPIO_Port, X_EN_Pin, GPIO_PIN_RESET); //Treiber aktivieren
	HAL_GPIO_WritePin(Z_EN_GPIO_Port, Z_EN_Pin, GPIO_PIN_RESET); //Treiber aktivieren

	/* UART Configuration */
	HAL_HalfDuplex_EnableReceiver(&huart2);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxDatagram, 32);

	//########################### Uart X (Uart2)
	uint8_t write[8] = { 0x05, 0x0, 0xA2, 0, 0, 0x0F, 0xFF, 0xE9 }; //Schreiben von VACTUAL
	uint8_t read[4] = { 0x05, 0x0, 0x2, 0x8F };	//Auslesen von IFCNT

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (BatteryAlarm) {
			//TODO gebe leeren Batteriestand auf Display aus
			//TODO Code in Interrupt stecken
			if (Move_To_Pos(HomePos))
				Error_Handler();
		}
		//*********************************************************************************************
		//TODO UART in seperate Datei
		write[7] = HAL_CRC_Calculate(&hcrc, (uint32_t *)write, sizeof(write)-1);
		read[3] = HAL_CRC_Calculate(&hcrc, (uint32_t *)read, sizeof(read)-1);
		//*********************************************************************************************
		HAL_HalfDuplex_EnableTransmitter(&huart2);
		HAL_UART_Transmit_DMA(&huart2, write, 8);
		HAL_HalfDuplex_EnableReceiver(&huart2);
		HAL_Delay(1000);
		HAL_HalfDuplex_EnableTransmitter(&huart2);
		HAL_UART_Transmit_DMA(&huart2, read, 4);
		HAL_HalfDuplex_EnableReceiver(&huart2);
		HAL_Delay(1000);
		//*********************************************************************************************
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

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = 64;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 34;
	RCC_OscInitStruct.PLL.PLLP = 1;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 3072;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
			|RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */
	__HAL_RCC_CRC_CLK_ENABLE();	//CRC-Clock aktivieren
	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
	hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
	hcrc.Init.GeneratingPolynomial = 7;
	hcrc.Init.CRCLength = CRC_POLYLENGTH_8B;
	hcrc.Init.InitValue = 0;
	hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
	hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
	hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

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
	htim2.Init.Prescaler = 274;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
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
	sConfigOC.Pulse = 500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief UART8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART8_Init(void)
{

	/* USER CODE BEGIN UART8_Init 0 */

	/* USER CODE END UART8_Init 0 */

	/* USER CODE BEGIN UART8_Init 1 */

	/* USER CODE END UART8_Init 1 */
	huart8.Instance = UART8;
	huart8.Init.BaudRate = 115200;
	huart8.Init.WordLength = UART_WORDLENGTH_8B;
	huart8.Init.StopBits = UART_STOPBITS_1;
	huart8.Init.Parity = UART_PARITY_NONE;
	huart8.Init.Mode = UART_MODE_TX_RX;
	huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart8.Init.OverSampling = UART_OVERSAMPLING_16;
	huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_HalfDuplex_Init(&huart8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART8_Init 2 */

	/* USER CODE END UART8_Init 2 */

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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
	/* DMA1_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	/* DMA1_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

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
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, Z_STEP_Pin|Z_DIR_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, X_DIR_Pin|X_STEP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(X_EN_GPIO_Port, X_EN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(HE0_PWM_GPIO_Port, HE0_PWM_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Z_EN_GPIO_Port, Z_EN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pins : Z_STEP_Pin Z_DIR_Pin Z_EN_Pin */
	GPIO_InitStruct.Pin = Z_STEP_Pin|Z_DIR_Pin|Z_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : PWRDET_Pin */
	GPIO_InitStruct.Pin = PWRDET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PWRDET_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Z_MIN_Pin X_MIN_Pin */
	GPIO_InitStruct.Pin = Z_MIN_Pin|X_MIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : X_DIR_Pin X_STEP_Pin X_EN_Pin */
	GPIO_InitStruct.Pin = X_DIR_Pin|X_STEP_Pin|X_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : HE0_PWM_Pin */
	GPIO_InitStruct.Pin = HE0_PWM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(HE0_PWM_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief GPIO External Interrupt Callback Function
 * @param GPIO_Pin GPIO-Pin with active Interrupt
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == PWRDET_Pin) {
		BatteryAlarm = true;
	}
}

/**
 * @brief Timer External Interrupt Callback Function
 * @param htim Pointer to Timer with elapsed Timer Callback
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		if (PWMEnabledX) {
			PWMCounterX++;
			if (PWMCounterX >= PWMPeriod) {
				PWMCounterX = 0;
				PWMStepX++;
			}
			if (PWMCounterX < PWMPulseWidth) {
				HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_RESET);
			}

			// Überprüfen, ob die Ziel-Schritte erreicht sind
			if (PWMStepX >= TargetStepsX) {
				PWMEnabledX = false;
				HAL_GPIO_WritePin(X_STEP_GPIO_Port, X_STEP_Pin, GPIO_PIN_RESET);
			}
		}
	}
}

/**
 * @brief UART Transmitt Completed Callback Function
 * @param huart Pointer to UART with completed Transmitt
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {

	}
}

/**
 * @brief UART Recive Completed Callback Function
 * @param huart Pointer to UART with recived data
 * @param Size Size of the recived data
 * @retval None
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rxDatagram, 32);
}
/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
	MPU_Region_InitTypeDef MPU_InitStruct = {0};

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
		HAL_Delay(100);
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
