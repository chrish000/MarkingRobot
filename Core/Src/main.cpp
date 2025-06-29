/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.cpp
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
/**
 * Wichtige Informationen für das Verständnis
 * 	>	Aufgrund der UART-Ansteuerung werden die Treiber X und Z benutzt.
 * 		Es werden trotzdem X-Y-Koordinaten verwendet.
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "move.h"
#include "pins.h"
#include "Buzzer/buzzer.h"
#include "Buzzer/buzzer_examples.h"
#include "homing.h"
#include <math.h>

#include "lcd/menu.h"
#include "lcd/lcd_bitmaps.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim23;
TIM_HandleTypeDef htim24;
DMA_HandleTypeDef hdma_tim23_ch1;
DMA_HandleTypeDef hdma_tim23_ch2;
DMA_HandleTypeDef hdma_tim24_ch1;
DMA_HandleTypeDef hdma_tim24_ch2;

UART_HandleTypeDef huart8;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart8_tx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* Peripherie */
ERROR_HandleCode ErrorCode = NONE;
extern Robot robi;

/* Sensorvariablen */
volatile uint8_t BatteryAlarm = false;
uint8_t PressureAlarm = false;
uint8_t printFlag = false;

uint8_t distSequence = 0;
uint8_t homingSequence = 0;
uint8_t airSequence = 0;
uint8_t readFromSD = true;
uint8_t homingRoutine = false;
uint8_t homingFailed = false;
uint8_t lowPressure;
uint8_t homingEnabledPressure = true;

Robot::MoveParams distHomingPosBuffer;

const MotorManager::position *currentPos = nullptr;
MotorManager::moveCommands tempCmdBuf[robi.motorMaster.buffer_size_move] = { };
MotorManager::position tempPosBuf[robi.motorMaster.buffer_size_move] = { };
uint8_t cmdCnt = 0, posCnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM23_Init(void);
static void MX_TIM24_Init(void);
static void MX_TIM4_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief ADC Conversion complete callback functipon.
 * 			Speichert Batteriespannung und Ladungszustand. Speichert Pinzustand von Druckalarm.
 * @param hadc Pointer zu ADC-Handler
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	robi.batteryVoltage = ((robi.ADC_BatteryVoltage / ADC_MAX) * V_REF)
			* VOLTAGE_DIVIDER_RATIO;
	robi.batteryPercentage = (uint8_t) ((robi.batteryVoltage - MIN_BAT_VOLTAGE)
			/ (MAX_BAT_VOLTAGE - MIN_BAT_VOLTAGE) * 100);
	lowPressure = !HAL_GPIO_ReadPin(PRESSURE_PORT, PRESSURE_PIN);
}
/**
 * @brief GPIO External Interrupt Callback Function
 * @param GPIO_Pin GPIO-Pin with active Interrupt
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case PWRDET_PIN:
		LowVoltageHandler();
		break;

	case X_STOP_PIN:
		xFlag = true;
		xDist = Dist;
		homingActive = !yFlag;
		break;

	case Y_STOP_PIN:
		yFlag = true;
		yDist = Dist;
		homingActive = !xFlag;
		break;

	case LCD_BTN_PIN:
		HAL_NVIC_DisableIRQ(LCD_BTN_EXTI); //wird in DisplayRoutine() wieder aktiviert
		menuIndex = selected;
		break;

	case LCD_ENCA_PIN:
		if (LCD_ENCB_PORT->IDR & LCD_ENCB_PIN)
			menuIndex = next;  // cw
		else
			menuIndex = prev;
		break;

	default:
		break;
	}
}

/**
 * @brief UART Transmitt Completed Callback Function
 * @param huart Pointer to UART with completed transmitt
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == robi.motorMaster.motorX.tmc.UART_address->Instance) {
		HAL_HalfDuplex_EnableReceiver(robi.motorMaster.motorX.tmc.UART_address);
		HAL_UART_Receive_DMA(robi.motorMaster.motorX.tmc.UART_address,
				robi.motorMaster.motorX.tmc.rxBufferRaw,
				TMC2209::WRITE_READ_REPLY_DATAGRAM_SIZE);
		robi.motorMaster.motorX.tmc.data_sent_flag = true;
	}
	if (huart->Instance == robi.motorMaster.motorY.tmc.UART_address->Instance) {
		HAL_HalfDuplex_EnableReceiver(robi.motorMaster.motorY.tmc.UART_address);
		HAL_UART_Receive_DMA(robi.motorMaster.motorY.tmc.UART_address,
				robi.motorMaster.motorY.tmc.rxBufferRaw,
				TMC2209::WRITE_READ_REPLY_DATAGRAM_SIZE);
		robi.motorMaster.motorY.tmc.data_sent_flag = true;
	}
}

/**
 * @brief UART Receive Completed Callback Function
 * @param huart Pointer to UART with received data
 * @param Size Size of the received data
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == robi.motorMaster.motorX.tmc.UART_address->Instance) {
		HAL_UART_Receive_DMA(robi.motorMaster.motorX.tmc.UART_address,
				robi.motorMaster.motorX.tmc.rxBufferRaw,
				TMC2209::WRITE_READ_REPLY_DATAGRAM_SIZE);
		robi.motorMaster.motorX.tmc.data_received_flag = true;
	}
	if (huart->Instance == robi.motorMaster.motorY.tmc.UART_address->Instance) {
		HAL_UART_Receive_DMA(robi.motorMaster.motorY.tmc.UART_address,
				robi.motorMaster.motorY.tmc.rxBufferRaw,
				TMC2209::WRITE_READ_REPLY_DATAGRAM_SIZE);
		robi.motorMaster.motorY.tmc.data_received_flag = true;
	}
}

/**
 * @brief DMA Finished Callback Function (DMA für Motoransteuerung)
 * @param hdma Poniter zu DMA-Handler
 * @retval None
 */
void DMA_Callback(DMA_HandleTypeDef *hdma) {
	StepperMotor *motor = nullptr;

	if (hdma->Instance == robi.motorMaster.motorX.TIM_DMA_BSRR->Instance) {
		motor = &robi.motorMaster.motorX;
		if (homingActive)
			++Dist;
	} else if (hdma->Instance
			== robi.motorMaster.motorY.TIM_DMA_BSRR->Instance) {
		motor = &robi.motorMaster.motorY;
	}

	if (motor->stepBuf.remove(&motor->StepCmdBuffer) == false) {
		motor->stopTimer();
	}
}

/**
 * @brief Timer period finished callback (z.B. Buzzer)
 * @param htim Poniter zu Timer-Handler
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == robi.pins.TIM_BUZZER->Instance) {
		HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
	}
}

/**
 * @brief Handle-Funktion für Batteriealarm
 * @param None
 * @retval None
 */
void LowVoltageHandler() {
	robi.motorMaster.moveBuf.remove(robi.motorMaster.buffer_size_move - 1);
	while (robi.motorMaster.calcInterval()) {
		if (robi.printhead.isActive() != printFlag) {
			printFlag ? robi.printhead.start() : robi.printhead.stop();
		}
	}
	robi.printhead.stop();
	robi.motorMaster.motorX.disableMotor();
	robi.motorMaster.motorX.stopTimer();
	robi.motorMaster.motorY.disableMotor();
	robi.motorMaster.motorY.stopTimer();
	robi.isHomedFlag = false;
	robi.printingFlag = false;
	HAL_TIM_Base_Stop(&htim8); // ADC
	HAL_NVIC_DisableIRQ(X_STOP_EXTI);
	HAL_NVIC_DisableIRQ(Y_STOP_EXTI);
	HAL_NVIC_DisableIRQ(LCD_BTN_EXTI);
	HAL_NVIC_DisableIRQ(LCD_ENCA_EXTI);
	HAL_NVIC_DisableIRQ(LCD_ENCB_EXTI);
	HAL_GPIO_WritePin(X_STOP_PORT, X_STOP_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Y_STOP_PORT, Y_STOP_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FAN0_PORT, FAN0_PIN, GPIO_PIN_RESET);

	UserErrorHandler(LOW_VOLTAGE);
}

/**
 * @brief Handle-Funktion für Error
 * @param errCode Error-Code für die Fallbehandlung
 * @retval None
 */
void UserErrorHandler(UserErrorCode errCode) {
	while (1) {
		switch (errCode) {
		case UNSPECIFIC:
			Buzzer_Note(&robi.hbuzzer, error_sound);
			break;
		case LOW_VOLTAGE:
			activeScreen = akku_leer;
			Buzzer_Note(&robi.hbuzzer, battery_empty);
			break;
		}
		DisplayRoutine();
	}
}

/**
 * @brief Handle-Funktion für niedrigen Druck bei Markiervorgang
 * @param None
 * @retval None
 */
void HandlePressureAlarm(void) {
	switch (airSequence) {
	case 0:
		readFromSD = false;
		homingEnabledPressure = false;

		//Aktuelle Position speichern
		currentPos = robi.motorMaster.posBuf.peek();
		if (currentPos != nullptr) {
			tempPosBuf[0] = *currentPos;
			posCnt = 1;
		}

		//Aktuelle Bewegung beenden
		while (!robi.motorMaster.moveCmdFinishedFlag
				&& !robi.motorMaster.motorX.stepBuf.isEmpty()) {
			robi.motorMaster.calcInterval();
			if (robi.printhead.isActive() != printFlag) {
				printFlag ? robi.printhead.start() : robi.printhead.stop();
			}
		}

		//Puffer zwischenspeichern
		while (robi.motorMaster.moveBuf.remove(tempCmdBuf[cmdCnt]))
			cmdCnt++;
		while (robi.motorMaster.posBuf.remove(tempPosBuf[posCnt]))
			posCnt++;

		//Aktuelle Position setzen
		if (currentPos != nullptr) {
			robi.setPos(currentPos->orient, currentPos->x, currentPos->y);
		}

		airSequence++;
		break;

	case 1:
		//Zu home fahren
		if (movementFinished(&robi)) {
			robi.moveToHome();
			while (!movementFinished(&robi))
				robi.motorMaster.calcInterval();
			activeScreen = druck_gering_abbrechen;
			Buzzer_Play_Song_Blocking(&robi.hbuzzer, air_empty,
					(sizeof(air_empty) / sizeof(air_empty[0])),
					BPM_SYSTEM_SOUND);
			airSequence++;
		}
		break;

	case 2:
		//Auf Auftanken warten und danach referenzieren
		DisplayRoutine();
		if (!lowPressure) {
			if (activeScreen == markieren_laeuft) {
				Buzzer_NoNote(&robi.hbuzzer);
				homingEnabledPressure = true;
				robi.isHomedFlag = false;
				menuIndex = undefined;
				airSequence++;
				break;
			}
		}
		break;

	case 3:
		if (robi.isHomedFlag) {
			homingEnabledPressure = false;
			airSequence++;
		}
		break;

	case 4:
		if (posCnt != 0) {
			//Zurück auf Position fahren
			Robot::MoveParams originalPosition;
			originalPosition.x = tempPosBuf[0].x;
			originalPosition.y = tempPosBuf[0].y;
			robi.moveToPos(originalPosition);
			while (!movementFinished(&robi))
				robi.motorMaster.calcInterval();

			//Originale Orientierung einnehmen
			float_t delta = fmod((tempPosBuf[0].orient - robi.getRot() + 540.0),
					360.0) - 180.0;
			robi.moveRot(delta, DEFAULT_SPEED, DEFAULT_ACCEL);
			while (!movementFinished(&robi))
				robi.motorMaster.calcInterval();

			//Puffer wiederherstellen
			robi.motorMaster.posBuf.consumerClear();
			robi.motorMaster.moveBuf.consumerClear();
			for (int i = 0; i < cmdCnt; ++i) {
				robi.motorMaster.moveBuf.insert(tempCmdBuf[i]);
			}
			for (int i = 0; i < posCnt; ++i) {
				robi.motorMaster.posBuf.insert(tempPosBuf[i]);
			}
			robi.setPos(tempPosBuf[0].orient, tempPosBuf[0].x, tempPosBuf[0].y);
		}
		cmdCnt = posCnt = 0;
		readFromSD = true;
		homingEnabledPressure = true;
		PressureAlarm = false;
		airSequence = 0;
		break;
	}
}

/**
 * @brief Handle-Funktion für erneutes Homing bei Markiervorgang
 * @param None
 * @retval None
 */
void HandleDistanceHoming(void) {
	switch (distSequence) {
	case 0:
		readFromSD = false;
		homingRoutine = true;
		distSequence++;
		break;
	case 1:
		if (!homingRoutine) {
			readFromSD = true;
			distSequence = 0;
		}
		break;
	}
}

/**
 * @brief Handle-Funktion für die Homing-Routine mit Anfahrt an Basis
 * @param None
 * @retval None
 */
void HandleHomingRoutine(void) {
	homingRoutine = true;
	switch (homingSequence) {
	case 0:
		// Bewegungspuffer abarbeiten
		while (robi.motorMaster.calcInterval()) {
			if (robi.printhead.isActive() != printFlag) {
				printFlag ? robi.printhead.start() : robi.printhead.stop();
			}
		}
		readFromSD = false;
		robi.printhead.stop();
		distHomingPosBuffer.x = robi.getPosX();
		distHomingPosBuffer.y = robi.getPosY();
		robi.moveToHome();
		while (robi.motorMaster.calcInterval())
			;
		if (lowPressure) {
			activeScreen = druck_gering_abbrechen;
			homingSequence = 2;
		} else
			homingSequence = 1;
		robi.isHomedFlag = false;
		homingSequence++;
		break;
	case 1: // Nur Homing
		DisplayRoutine();
		if (robi.isHomedFlag)
			homingSequence = 3;
		break;
	case 2: // Luft und dann Homing
		DisplayRoutine();
		if (!lowPressure)
			if (activeScreen == markieren_laeuft) {
				robi.isHomedFlag = false;
				homingSequence = 1;
			}
		Buzzer_Play_Song(&robi.hbuzzer, air_empty,
				(sizeof(air_empty) / sizeof(air_empty[0])),
				BPM_SYSTEM_SOUND);
		menuIndex = undefined;
		break;
	case 3: // Fortsetzen
		robi.moveToPos(distHomingPosBuffer);
		robi.totalDistSinceHoming = 0;
		homingSequence = 0;
		homingRoutine = false;
		break;
	}
}

/**
 * @brief Handle-Funktion Homing-Vorgang
 * @param None
 * @retval None
 */
void HandleHoming(void) {
	readFromSD = false;
	timeoutActive = false;
	counterTry = 0;
	HOMING_StatusTypeDef status = HOMING_BUSY;
	while (status == HOMING_BUSY) {
		DisplayRoutine();
		status = home(&robi);
		robi.motorMaster.calcInterval();
	}
	if (status != HOMING_FINISHED) {
		homingFailed = true;
		activeScreen = referenzieren_gescheitert_abbrechen;
		DisplayRoutine();
		robi.printingFlag = false; //wird in menu.cpp wieder gesetzt
	} else {
		homingFailed = false;
		readFromSD = true;
	}
}

/**
 * @brief Handle-Funktion "Fertig"-Ereignis, ausgelöst durch M5 G-Code
 * @param None
 * @retval None
 */
void HandlePrintFinished(void) {
	if (movementFinished(&robi)) {
		robi.printhead.stop();
		robi.moveToHome();
		while (!movementFinished(&robi))
			robi.motorMaster.calcInterval();
		robi.motorMaster.motorX.disableMotor();
		robi.motorMaster.motorY.disableMotor();
		robi.finishedFlag = false;
		robi.printingFlag = false;
		robi.sd.closeCurrentFile();
		activeScreen = markieren_beendet;
		DisplayRoutine();

		/*
		Buzzer_Play_Song_Blocking(&robi.hbuzzer, mario_level_complete,
				(sizeof(mario_level_complete) / sizeof(mario_level_complete[0])),
				BPM_MARIO_LEVEL);
				*/
		Buzzer_NoNote(&robi.hbuzzer);
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_UART8_Init();
	MX_USART2_UART_Init();
	MX_CRC_Init();
	MX_ADC1_Init();
	MX_TIM8_Init();
	MX_TIM23_Init();
	MX_TIM24_Init();
	MX_TIM4_Init();
	MX_SPI1_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */
	robi.init();
	MX_U8G2_Init();

	/*
	 Buzzer_Play_Song_Blocking(&robi.hbuzzer, mario_theme,
	 (sizeof(mario_theme) / sizeof(mario_theme[0])), BPM_MARIO);
	 */

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* Druckvorgang */
		if (robi.printingFlag) {

			//lowPressure = !HAL_GPIO_ReadPin(PRESSURE_PORT, PRESSURE_PIN); verlagert in ADC Callback

			//UpdateStatusBarFast(); begrenzt maximal mögliche geschw.

			/* Referenzierung auslösen wenn Strecke erreicht */
			/*
			if (robi.totalDistSinceHoming > DIST_TILL_NEW_HOMING || homingRoutine) {
				HandleHomingRoutine();
			}
			*/

			/* Routine für Homing (Referenzierung) während des Markiervorgangs */
			/*
			if (homingRoutine) { //aktiviert durch HandleDistanceHoming()
				HandleHomingRoutine();
			}
			*/

			/* Zu niedriger Druck */
			if ((lowPressure && !homingRoutine) || PressureAlarm) {
				PressureAlarm = true;
				HandlePressureAlarm();
			}

			/*
			if (PressureAlarm) { //TODO Prüfen, ob in jeder Situation korrekt ausgeführt
				HandlePressureAlarm();
			}
			*/

			/* Roboter neu referenzieren wenn gefordert */
			if (!robi.isHomedFlag && !homingFailed && homingEnabledPressure) {
				HandleHoming();
			}

			/* Düse ansteuern*/
			if (robi.printhead.isActive() != printFlag) {
				printFlag ? robi.printhead.start() : robi.printhead.stop();
			}

			/* Befehl aus SD lesen */
			if (robi.motorMaster.moveBuf.writeAvailable() >= 2 && readFromSD) {
				if (robi.sd.readNextLine())
					robi.parser.parseGCodeLineAndPushInBuffer(
							robi.sd.lineBuffer);
			}

			/* Ende von Druckvorgang */
			if (robi.finishedFlag) {
				HandlePrintFinished();
			}

			/* Motoren ansteuern */
			robi.motorMaster.calcInterval();

		} else
			/* Menue */
			DisplayRoutine();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
	PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_16B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T8_TRGO;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.Oversampling.Ratio = 1;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_16;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void) {

	/* USER CODE BEGIN CRC_Init 0 */

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
	if (HAL_CRC_Init(&hcrc) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 0x0;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi1.Init.TxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi1.Init.RxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 27500 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000 - 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	__HAL_TIM_DISABLE_OCxPRELOAD(&htim2, TIM_CHANNEL_2);
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 27500 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000 - 1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	__HAL_TIM_DISABLE_OCxPRELOAD(&htim3, TIM_CHANNEL_1);
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 275 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0xffff;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 27500 - 1;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 5000 - 1;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */

}

/**
 * @brief TIM23 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM23_Init(void) {

	/* USER CODE BEGIN TIM23_Init 0 */

	/* USER CODE END TIM23_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM23_Init 1 */

	/* USER CODE END TIM23_Init 1 */
	htim23.Instance = TIM23;
	htim23.Init.Prescaler = 0;
	htim23.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim23.Init.Period = 0xffffffff;
	htim23.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim23.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim23) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim23, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim23) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim23, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim23, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim23, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM23_Init 2 */

	/* USER CODE END TIM23_Init 2 */

}

/**
 * @brief TIM24 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM24_Init(void) {

	/* USER CODE BEGIN TIM24_Init 0 */

	/* USER CODE END TIM24_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM24_Init 1 */

	/* USER CODE END TIM24_Init 1 */
	htim24.Instance = TIM24;
	htim24.Init.Prescaler = 0;
	htim24.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim24.Init.Period = 0xffffffff;
	htim24.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim24.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim24) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim24, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim24) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim24, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim24, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_ConfigChannel(&htim24, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM24_Init 2 */

	/* USER CODE END TIM24_Init 2 */

}

/**
 * @brief UART8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART8_Init(void) {

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
	if (HAL_HalfDuplex_Init(&huart8) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_HalfDuplex_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

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
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	/* DMA1_Stream7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
			Z_STEP_Pin | Z_DIR_Pin | LCD_CS_Pin | LCD_D4_Pin | LCD_D5_Pin
					| LCD_D6_Pin | LCD_D7_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LCD_EN_Pin | FAN2_Pin | FAN1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, X_DIR_Pin | X_STEP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(X_EN_GPIO_Port, X_EN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(FAN0_GPIO_Port, FAN0_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Z_EN_GPIO_Port, Z_EN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : Z_STEP_Pin */
	GPIO_InitStruct.Pin = Z_STEP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(Z_STEP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Z_DIR_Pin */
	GPIO_InitStruct.Pin = Z_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(Z_DIR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PWRDET_Pin */
	GPIO_InitStruct.Pin = PWRDET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(PWRDET_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : X_STOP_Pin Y_STOP_Pin */
	GPIO_InitStruct.Pin = X_STOP_Pin | Y_STOP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PRESSURE_Pin */
	GPIO_InitStruct.Pin = PRESSURE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(PRESSURE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SD_CS_Pin */
	GPIO_InitStruct.Pin = SD_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SD_DET_Pin */
	GPIO_InitStruct.Pin = SD_DET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SD_DET_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : BUZZER_Pin */
	GPIO_InitStruct.Pin = BUZZER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_BTN_Pin LCD_ENCB_Pin */
	GPIO_InitStruct.Pin = LCD_BTN_Pin | LCD_ENCB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_EN_Pin */
	GPIO_InitStruct.Pin = LCD_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LCD_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_ENCA_Pin */
	GPIO_InitStruct.Pin = LCD_ENCA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(LCD_ENCA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_CS_Pin LCD_D4_Pin */
	GPIO_InitStruct.Pin = LCD_CS_Pin | LCD_D4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
	GPIO_InitStruct.Pin = LCD_D5_Pin | LCD_D6_Pin | LCD_D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : X_DIR_Pin */
	GPIO_InitStruct.Pin = X_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(X_DIR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : X_STEP_Pin */
	GPIO_InitStruct.Pin = X_STEP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(X_STEP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : X_EN_Pin */
	GPIO_InitStruct.Pin = X_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(X_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : FAN2_Pin FAN1_Pin FAN0_Pin */
	GPIO_InitStruct.Pin = FAN2_Pin | FAN1_Pin | FAN0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : Z_EN_Pin */
	GPIO_InitStruct.Pin = Z_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Z_EN_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(LCD_BTN_EXTI_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(LCD_BTN_EXTI_IRQn);

	HAL_NVIC_SetPriority(X_STOP_EXTI_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(X_STOP_EXTI_IRQn);

	HAL_NVIC_SetPriority(LCD_ENCB_EXTI_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(LCD_ENCB_EXTI_IRQn);

	HAL_NVIC_SetPriority(Y_STOP_EXTI_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(Y_STOP_EXTI_IRQn);

	HAL_NVIC_SetPriority(LCD_ENCA_EXTI_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(LCD_ENCA_EXTI_IRQn);

	HAL_NVIC_SetPriority(PWRDET_EXTI_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(PWRDET_EXTI_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();

	while (1) {
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
