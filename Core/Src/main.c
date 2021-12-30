/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "gm65.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	QR_START				0
#define	QR_SETUP_OFF			1
#define QR_SETUP_OFF_T			11
#define QR_SETUP_OFF_R			12
#define	QR_IDLE					2
#define	QR_SCAN					3
#define QR_SCAN_T				31
#define	QR_SCAN_RC				32
#define	QR_SCAN_RH				33
#define QR_SCAN_RD				34
#define	QR_SCAN_SETUP_OFF		4
#define QR_SCAN_SETUP_OFF_T		41
#define	QR_SCAN_SETUP_OFF_R		42
#define	QR_ABORT				5

#define QR_LENGTH_COMMAND				9
#define QR_LENGTH_RESPOND_COMMAND		7
#define QR_LENGTH_HEAD					3

#define QR_COMMAND_WAIT					5000
#define QR_SCAN_WAIT					55000

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */
uint8_t qrRespuestaValida(uint8_t *respuesta);
uint8_t qrRespSetupOnCode(uint8_t *code);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint8_t dato[4];
uint8_t qrSetupOFF[] ={ 	0x7E, 0,
								0x08,
								0x01,
								0x00, 0x03,
								0x03,
								0x11, 0xA9};
uint8_t qrWriteOK[]={		0x02, 0x00,
								0x00,
								0x01,
								0x00,
								0x33, 0x31};
uint8_t qrScan[]={		0x7E, 0x00,
								0x08,
								0x01,
								0x00, 0x02,
								0x01,
								0xAB, 0xCD};
uint8_t qrSetupOnCode[]={	0x51, 0x80, 0x52, 0x30, 0x33, 0x30, 0x32, 0x30, 0x30, 0x2E};
uint8_t qrBuffer[256];
uint8_t qrLengthDecode = 0;
uint8_t qrStatus = QR_START;

uint8_t qrRequest = 0;
uint8_t qrDataReady = 0;
char texto[25];
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
  MX_USART3_UART_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  //HAL_Delay(1000);


  HAL_UART_Transmit_IT(&huart3, qrSetupOFF, QR_LENGTH_COMMAND);
  qrStatus= QR_SETUP_OFF_T;
  __HAL_TIM_SET_COUNTER(&htim9, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(qrDataReady==1){
		  sprintf(texto,"Codigo decodificado:\n");
		  HAL_UART_Transmit(&huart2, (uint8_t*)texto, 21 , HAL_MAX_DELAY);
		  if(qrBuffer[0] ==0x51 && qrBuffer[qrLengthDecode-1]== 0x0D){
			  HAL_UART_Transmit(&huart2, qrBuffer+1, qrLengthDecode-2, HAL_MAX_DELAY);
		  }
		  else if(qrBuffer[0] != 0x51){
			  sprintf(texto,"No es QR\n");
			  HAL_UART_Transmit(&huart2, (uint8_t*)texto, 9 , HAL_MAX_DELAY);
		  }
		  else{
			  sprintf(texto,"ERROR\n");
			  HAL_UART_Transmit(&huart2, (uint8_t*)texto, 6 , HAL_MAX_DELAY);
		  }
		  qrDataReady=0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 16799;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 4000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */
  __HAL_TIM_CLEAR_FLAG(&htim9, TIM_FLAG_UPDATE);
  __HAL_TIM_CLEAR_IT(&htim9, TIM_IT_UPDATE);
  /* USER CODE END TIM9_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3){
		HAL_TIM_Base_Stop_IT(&htim9);
		switch (qrStatus){
		case QR_SETUP_OFF_T:
			HAL_UART_Receive_IT(&huart3, qrBuffer, QR_LENGTH_RESPOND_COMMAND);
			__HAL_TIM_SET_AUTORELOAD(&htim9, QR_COMMAND_WAIT);
			__HAL_TIM_SET_COUNTER(&htim9, 0);
			HAL_TIM_Base_Start_IT(&htim9);
			qrStatus=QR_SETUP_OFF_R;
			break;
		case QR_SCAN_T:
			HAL_UART_Receive_IT(&huart3, qrBuffer, QR_LENGTH_RESPOND_COMMAND);
			__HAL_TIM_SET_AUTORELOAD(&htim9, QR_COMMAND_WAIT);
			__HAL_TIM_SET_COUNTER(&htim9, 0);
			HAL_TIM_Base_Start_IT(&htim9);
			qrStatus=QR_SCAN_RC;
			break;
		case QR_SCAN_SETUP_OFF_T:
			HAL_UART_Receive_IT(&huart3, qrBuffer, QR_LENGTH_RESPOND_COMMAND);
			__HAL_TIM_SET_AUTORELOAD(&htim9, QR_COMMAND_WAIT);
			__HAL_TIM_SET_COUNTER(&htim9, 0);
			HAL_TIM_Base_Start_IT(&htim9);
			qrStatus=QR_SCAN_SETUP_OFF_R;
			break;
		default:
			break;
		}
		}

	if(huart->Instance == USART2){
		//HAL_UART_Transmit_IT(&huart2, dato, 4);
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3){
			HAL_TIM_Base_Stop_IT(&htim9);
			__HAL_TIM_SET_COUNTER(&htim9,0);
			switch (qrStatus){
			case QR_SETUP_OFF_R:
				if(qrRespuestaValida(qrBuffer) == 1){
					//HAL_Delay(1000);
					HAL_UART_Transmit_IT(&huart3, qrSetupOFF, QR_LENGTH_COMMAND);
					qrStatus=QR_SETUP_OFF_T;

				}
				else{
					qrStatus=QR_START;
				}
				break;

			case QR_SCAN_RC:
				if(qrRespuestaValida(qrBuffer) == 1){
					//HAL_Delay(1000);
					HAL_UART_Transmit_IT(&huart3, qrScan, QR_LENGTH_COMMAND);
					qrStatus=QR_SCAN_T;
				}
				else{
					qrStatus=QR_SCAN_RH;
					HAL_UART_Receive_IT(&huart3, qrBuffer, QR_LENGTH_HEAD);
					__HAL_TIM_SET_AUTORELOAD(&htim9, QR_SCAN_WAIT);
					HAL_TIM_Base_Start_IT(&htim9);
				}
				break;
			case QR_SCAN_RH:
				if((qrBuffer[0]=0x03) && (qrBuffer[1] == 0x00)){
					qrLengthDecode = qrBuffer[2];
					HAL_UART_Receive_IT(&huart3, qrBuffer, qrLengthDecode);
					__HAL_TIM_SET_AUTORELOAD(&htim9, QR_SCAN_WAIT);
					HAL_TIM_Base_Start_IT(&htim9);
					qrStatus=QR_SCAN_RD;
				}
				else{
					//HAL_Delay(1000);
					HAL_UART_Transmit_IT(&huart3, qrScan, QR_LENGTH_COMMAND);
					qrStatus=QR_SCAN_T;
				}
				break;
			case QR_SCAN_RD:
				if((qrLengthDecode == 0x0B) && (qrRespSetupOnCode(qrBuffer)==0)){
					qrStatus=QR_SCAN_SETUP_OFF_T;
					HAL_UART_Transmit_IT(&huart3, qrSetupOFF, QR_LENGTH_COMMAND);
				}
				else{
					qrDataReady=1;
					qrStatus=QR_START;
				}
				break;
			case QR_SCAN_SETUP_OFF_R:
				if(qrRespuestaValida(qrBuffer) == 1){
					//HAL_Delay(1000);
					HAL_UART_Transmit_IT(&huart3, qrSetupOFF, QR_LENGTH_COMMAND);
					qrStatus=QR_SCAN_SETUP_OFF_T;
				}
				else{
					qrStatus=QR_SCAN_T;
					HAL_UART_Transmit_IT(&huart3, qrScan, QR_LENGTH_COMMAND);
				}
				break;
			default:
				break;
			}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(qrStatus == QR_START){
		qrStatus=QR_SCAN_T;
		HAL_UART_Transmit_IT(&huart3, qrScan, QR_LENGTH_COMMAND);
	}
}

uint8_t qrRespuestaValida(uint8_t *respuesta){
	uint8_t flag = 0;
	for(int i=0; i<7; i++){
		if(*(respuesta+i) != *(qrWriteOK+i)){
			flag = 1;
			break;
		}
	}
	return flag;
}


uint8_t qrRespSetupOnCode(uint8_t *code){
	uint8_t flag = 0;
	for(int i=0; i<10; i++){
		if(*(code+i) != *(qrSetupOnCode+i)){
			flag = 1;
			break;
		}
	}
	return flag;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance ==TIM9){
		HAL_UART_AbortReceive_IT(&huart3);
		HAL_TIM_Base_Stop_IT(&htim9);
		__HAL_TIM_SET_COUNTER(&htim9, 0);
		switch (qrStatus){
		case QR_SETUP_OFF_R:
			qrStatus=QR_SETUP_OFF_T;
			HAL_UART_Transmit_IT(&huart3, qrSetupOFF, 9);
			break;
		case QR_SCAN_RC:
			HAL_UART_Transmit_IT(&huart3, qrScan, 9);
			qrStatus=QR_SCAN_T;
			break;
		case QR_SCAN_RH:
			qrStatus=QR_SCAN_T;
			HAL_UART_Transmit_IT(&huart3, qrScan, 9);
			break;
		case QR_SCAN_RD:
			qrStatus=QR_SCAN_T;
			HAL_UART_Transmit_IT(&huart3, qrScan, 9);
			break;
		case QR_SCAN_SETUP_OFF_R:
			qrStatus=QR_SCAN_SETUP_OFF_T;
			HAL_UART_Transmit_IT(&huart3, qrSetupOFF, 9);
		default:
			break;
		}
	}
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
  return;
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

