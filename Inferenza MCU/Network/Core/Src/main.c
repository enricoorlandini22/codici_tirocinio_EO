/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "network.h"
#include "network_data.h"
#include "ai_platform.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define F_OUT "output.bin" //file in cui vengono salvati gli output della rete
#define F_REPORT "report.txt" //file per riportare i tempi di inferenza
#define F_INPUT "input.bin" //file contenente il vettore di input

typedef ai_float IN_TYPE;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

SD_HandleTypeDef hsd1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// utilizzo struttura FIL di FATFS per gestire i file
FIL file_out;
FIL file_in;
FIL f_report;
char buffer[80];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

int ai_run(const void*, void*);
int ai_init(void);
int network_run(IN_TYPE* in_data, IN_TYPE* out_data, IN_TYPE* ingresso); //chiama f. inferenze e gestisce output
void set_input_data(IN_TYPE* in_data, int k, IN_TYPE* ingresso); //per riempire ingressi rete
int process_output(IN_TYPE* out_data);
FRESULT read_from_sd(long int, ai_float*, int);
void write_to_sd(int);
void write_report(char*, int);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//inizializzazione istanza della rete neurale
static ai_handle network = AI_HANDLE_NULL;

/* Array per gestire il buffer delle attivazioni */
AI_ALIGNED(32)
static ai_u8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

/* Array per gestire il tensore di input */
AI_ALIGNED(32)
static IN_TYPE in_data[AI_NETWORK_IN_1_SIZE];


/* Array per gestire il tensore di output */
AI_ALIGNED(32)
static IN_TYPE out_data[AI_NETWORK_OUT_1_SIZE];


/*Array di puntatori per gestire i tensori di input/output */
static ai_buffer *ai_input;
static ai_buffer *ai_output;

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
  MX_CRC_Init();
  MX_SDMMC1_SD_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  //creo un instanza della rete  e degli opportuni buffer per ingressi e uscite
  if (ai_init() != 0)
     	  {
     		  Error_Handler();
     	  }
  FRESULT res = FR_OK;

  //monto la scheda SD
  res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
      if(res != FR_OK)
      {
       	  	   	Error_Handler();
      }

   strcpy (buffer, "Rete neurale: ");
   write_report (buffer, strlen(buffer));

   int string_len = strlen(AI_NETWORK_ORIGIN_MODEL_NAME);
   write_report(AI_NETWORK_ORIGIN_MODEL_NAME, string_len);

   res = f_open(&file_in, F_INPUT, FA_READ);
   if(res != FR_OK)
   {
   Error_Handler();
   }
   //salvo la dimensione dell'intero file di input
   long int file_length = f_size(&file_in);
   res = f_close(&file_in);
   if(res != FR_OK)
   {
   Error_Handler();
   }

   //calcolo il numero di valori (float32) contenuti nel file di input
   long int n_valori = file_length / sizeof(ai_float);

   //dimensione di ogni blocco in cui suddivido il file input da leggere
   long int size_blocco = 10000;

   //numero di blocchi in cui suddivido il file input
   int b_max = n_valori / size_blocco;
   int blocco_attuale = 0;

   ai_float* ingresso;


   strcpy (buffer, "\nCalcolo tempi per ogni inferenza:\n");
   string_len = strlen(buffer);
   write_report(buffer, string_len);

   while(blocco_attuale < b_max)
   {
	   //vado ad allocare lo spazio per un blocco, sul quale farò le inferenze
	   ingresso = malloc(sizeof(ai_float) * size_blocco);

	   //il blocco di valori è letto dal file di input nella scheda SD
	   //mi posiziono nel punto opportuno definito da blocco_attuale
	   res = read_from_sd(size_blocco, ingresso, blocco_attuale);
	    	  if(res != FR_OK)
	    	  {
	       	    	  Error_Handler();
	    	  }

	   //ciclo su tutto il blocco corrente
	   //attenzione: bisogna passare alla rete 4 valori alla volta
	   //senza sovrapposizioni e senza perdere dati

	   int k = 0; //indice del valore letto
	   while (k < size_blocco)
	   {


		   set_input_data(in_data, k, ingresso);

		   if (network_run(in_data, out_data, ingresso) != 0)
		    			  Error_Handler();

		   //scorro al prossimo array di ingresso per la rete
		   k = k + AI_NETWORK_IN_1_SIZE;

	   }

	   free(ingresso); //libero lo spazio di memoria allocato per questo blocco terminato
	   blocco_attuale++; //passo al blocco successivo

   }

   //se sono arrivato all'ultimo blocco, che contiene meno valori della dimensione predefinita
   //oppure l'intero file di input è più piccolo della dimensione predefinita
   if(((blocco_attuale) == b_max)||(b_max == 0))
   {
	   n_valori = n_valori - (b_max * size_blocco);
	   ingresso = malloc(sizeof(ai_float)* n_valori);
	   res = read_from_sd(n_valori, ingresso, blocco_attuale);
	   if(res != FR_OK)
	   	  {
	   		  Error_Handler();
	   	  }
	   int k = 0; //indice del valore di partenza dell'array letto
	   while (k < size_blocco)
	   {
	   		if (network_run(in_data, out_data, ingresso) != 0)
	   		    			  Error_Handler();

	   		//scorro al prossimo array di ingresso per la rete
	   		k = k + AI_NETWORK_IN_1_SIZE;
       }

	   free(ingresso);


   }

   strcpy (buffer,"\nFine\n");
   string_len = strlen(buffer);
   write_report(buffer, string_len);

   f_mount(&SDFatFS, (TCHAR const*)NULL, 0);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
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
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  hsd1.Init.Transceiver = SDMMC_TRANSCEIVER_DISABLE;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 120;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED1_Pin|DCDC_2_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED2_Pin|WIFI_WAKEUP_Pin|CS_DH_Pin|EX_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, WIFI_RST_Pin|SPI2_MISO_p2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, CS_WIFI_Pin|C_EN_Pin|CS_ADWB_Pin|STSAFE_RESET_Pin
                          |WIFI_BOOT0_Pin|CS_DHC_Pin|SEL3_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, BLE_SPI_CS_Pin|SEL1_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_MOSI_p2_Pin|PB11_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BOOT0_PE0_Pin BLE_TEST8_Pin */
  GPIO_InitStruct.Pin = BOOT0_PE0_Pin|BLE_TEST8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB9_Pin PB8_Pin PB14_Pin CHRGB0_Pin */
  GPIO_InitStruct.Pin = PB9_Pin|PB8_Pin|PB14_Pin|CHRGB0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT0_PE0H3_Pin */
  GPIO_InitStruct.Pin = BOOT0_PE0H3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT0_PE0H3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI3_MISO_Pin SPI3_MOSI_Pin SPI3_CLK_Pin */
  GPIO_InitStruct.Pin = SPI3_MISO_Pin|SPI3_MOSI_Pin|SPI3_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_MISO_Pin SPI2_CLK_Pin */
  GPIO_InitStruct.Pin = SPI2_MISO_Pin|SPI2_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_TEST9_Pin WIFI_DRDY_Pin INT1_DHC_Pin INT_STT_Pin
                           INT1_ADWB_Pin */
  GPIO_InitStruct.Pin = BLE_TEST9_Pin|WIFI_DRDY_Pin|INT1_DHC_Pin|INT_STT_Pin
                          |INT1_ADWB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : EX_PWM_Pin */
  GPIO_InitStruct.Pin = EX_PWM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(EX_PWM_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_DP_Pin OTG_FS_DM_Pin */
  GPIO_InitStruct.Pin = OTG_FS_DP_Pin|OTG_FS_DM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SAI1_SCK_A_Pin SAI1_MCLK_A_Pin SAI1_FS_A_DFSDM_D3_Pin SAI1_SD_A_Pin
                           SAI1_SD_B_Pin */
  GPIO_InitStruct.Pin = SAI1_SCK_A_Pin|SAI1_MCLK_A_Pin|SAI1_FS_A_DFSDM_D3_Pin|SAI1_SD_A_Pin
                          |SAI1_SD_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin DCDC_2_EN_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|DCDC_2_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin WIFI_WAKEUP_Pin CS_DH_Pin EX_RESET_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|WIFI_WAKEUP_Pin|CS_DH_Pin|EX_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10_Pin PA9_Pin PA0_Pin DAC1_OUT1_Pin
                           PA1_Pin */
  GPIO_InitStruct.Pin = PA10_Pin|PA9_Pin|PA0_Pin|DAC1_OUT1_Pin
                          |PA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DFSDM1_DATIN5_Pin DFSDM1_D7_Pin */
  GPIO_InitStruct.Pin = DFSDM1_DATIN5_Pin|DFSDM1_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG12_Pin PG10_Pin PG9_Pin */
  GPIO_InitStruct.Pin = PG12_Pin|PG10_Pin|PG9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_RST_Pin */
  GPIO_InitStruct.Pin = BLE_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : WIFI_RST_Pin SPI2_MISO_p2_Pin */
  GPIO_InitStruct.Pin = WIFI_RST_Pin|SPI2_MISO_p2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C2_SMBA_Pin I2C2_SDA_Pin I2C2_SDAF0_Pin */
  GPIO_InitStruct.Pin = I2C2_SMBA_Pin|I2C2_SDA_Pin|I2C2_SDAF0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_WIFI_Pin C_EN_Pin CS_ADWB_Pin STSAFE_RESET_Pin
                           WIFI_BOOT0_Pin CS_DHC_Pin SEL3_4_Pin */
  GPIO_InitStruct.Pin = CS_WIFI_Pin|C_EN_Pin|CS_ADWB_Pin|STSAFE_RESET_Pin
                          |WIFI_BOOT0_Pin|CS_DHC_Pin|SEL3_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C3_SDA_Pin I2C3_SCL_Pin */
  GPIO_InitStruct.Pin = I2C3_SDA_Pin|I2C3_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_SEL_Pin */
  GPIO_InitStruct.Pin = SW_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
  HAL_GPIO_Init(SW_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INT2_DHC_Pin PGOOD_Pin INT_M_Pin */
  GPIO_InitStruct.Pin = INT2_DHC_Pin|PGOOD_Pin|INT_M_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_MISO_Pin SPI1_MOSI_Pin SPI1_CLK_Pin */
  GPIO_InitStruct.Pin = SPI1_MISO_Pin|SPI1_MOSI_Pin|SPI1_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : BLE_SPI_CS_Pin SEL1_2_Pin */
  GPIO_InitStruct.Pin = BLE_SPI_CS_Pin|SEL1_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : INT_HTS_Pin BLE_INT_Pin */
  GPIO_InitStruct.Pin = INT_HTS_Pin|BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C4_SCL_Pin I2C4_SDA_Pin */
  GPIO_InitStruct.Pin = I2C4_SCL_Pin|I2C4_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : ADC1_IN1_Pin ADC1_IN2_Pin uC_ADC_BATT_Pin */
  GPIO_InitStruct.Pin = ADC1_IN1_Pin|ADC1_IN2_Pin|uC_ADC_BATT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INT2_ADWB_Pin SD_DETECT_Pin */
  GPIO_InitStruct.Pin = INT2_ADWB_Pin|SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CHRG_Pin */
  GPIO_InitStruct.Pin = CHRG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHRG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_PWR_Pin */
  GPIO_InitStruct.Pin = BUTTON_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_PWR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART3_RX_Pin USART3_TX_Pin */
  GPIO_InitStruct.Pin = USART3_RX_Pin|USART3_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI2_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(SPI2_MOSI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART3_RTS_Pin USART3_CTS_Pin */
  GPIO_InitStruct.Pin = USART3_RTS_Pin|USART3_CTS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DFSDM1_CKOUT_Pin */
  GPIO_InitStruct.Pin = DFSDM1_CKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(DFSDM1_CKOUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_MOSI_p2_Pin PB11_Pin */
  GPIO_InitStruct.Pin = SPI2_MOSI_p2_Pin|PB11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INT2_DH_Pin */
  GPIO_InitStruct.Pin = INT2_DH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT2_DH_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EX_ADC_Pin */
  GPIO_InitStruct.Pin = EX_ADC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EX_ADC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE12_Pin */
  GPIO_InitStruct.Pin = PE12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PE12_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int ai_init(void) {
  ai_error error;

  /* Create and initialize the c-model */
  const ai_handle acts[] = { activations };
  error = ai_network_create_and_init(&network, acts, NULL);
  if (error.type != AI_ERROR_NONE) {
	  Error_Handler();
  }

  ai_input = ai_network_inputs_get(network, NULL);
  ai_output = ai_network_outputs_get(network, NULL);

  return 0;
}



int ai_run(const void *in_data, void *out_data) {

	ai_i32 n_batch;
	ai_error error;

	ai_input[0].data = AI_HANDLE_PTR(in_data);
	ai_output[0].data = AI_HANDLE_PTR(out_data);
/*run*/
	n_batch = ai_network_run(network, &ai_input[0], &ai_output[0]);
	if (n_batch != 1) {
		error = ai_network_get_error(network);
		Error_Handler();
	};

	return 0;
}

void set_input_data(IN_TYPE* in_data, int k, IN_TYPE* ingresso)
{
	    //vado a riempire l'effettivo array di input da passare alla rete
		for (int i = 0; i < AI_NETWORK_IN_1_SIZE; i++)
			in_data[i] = ingresso[i+k];
		//in_data è lungo AI_MODEL_IN_1_SIZE e dipende da k
		// ovvero da dove sono posizionato del blocco corrente

}

int process_output(IN_TYPE* out_data) {
    // Applica la softmax
    ai_float exp0 = exp(out_data[0]);
    ai_float exp1 = exp(out_data[1]);
    ai_float sum_exp = exp0 + exp1;

    ai_float softmax1 = exp1 / sum_exp;  // Probabilità per la classe 1

    // Determina il risultato
    int risultato = (softmax1 > 0.5f) ? 1 : 0;  // 0.5 è la soglia tipica

    // Scrivi il risultato nel file di output
    return risultato;
}

int network_run(IN_TYPE* in_data, IN_TYPE* out_data, IN_TYPE* ingresso)
{
    uint16_t time;

    // Avvia il timer per misurare il tempo di inferenza
    HAL_TIM_Base_Start(&htim1);

    time = __HAL_TIM_GET_COUNTER(&htim1);

    // Esegui la rete neurale
    ai_run(in_data, out_data);

    // Calcola il tempo di inferenza
    time = __HAL_TIM_GET_COUNTER(&htim1) - time;

    // Scrivi il report del tempo di inferenza
    sprintf(buffer, "tempo inferenza : %uus\n", time);
    int lunghezza_stringa = strlen(buffer);
    write_report(buffer, lunghezza_stringa);


    // Determina l'etichetta (0 o 1) in base al risultato
    int risultato = process_output(out_data);

    //aggiungo il risultato dell'inferenza al file di output
    write_to_sd(risultato);

    return 0;
}

FRESULT read_from_sd(long int size_blocco, ai_float* valori, int blocco)
{
    FRESULT res;

    // Apro il file in modalità lettura
    res = f_open(&file_in, F_INPUT, FA_READ);
    if (res != FR_OK) {
        Error_Handler();  // Gestione errore apertura file
        return res;
    }

    // Calcolo l'offset per posizionarmi nel file di ingresso secondo il blocco in cui mi trovo
    // size_blocco definisce la dimensione del blocco
    int offset = blocco * size_blocco * sizeof(ai_float);

    // Posiziono il puntatore del file all'offset calcolato
    res = f_lseek(&file_in, offset);
    if (res != FR_OK) {
        Error_Handler();  // Gestione errore
        return res;
    }

    // Leggiamo i 'chunk_size' valori dal file
    UINT br;
    res = f_read(&file_in, valori, sizeof(ai_float) * size_blocco, &br);
    if (res != FR_OK || br != (sizeof(ai_float) * size_blocco)) {
        Error_Handler();  // Gestione errore durante la lettura
        return res;
    }

    // Chiudiamo il file
    res = f_close(&file_in);
    if (res != FR_OK) {
        Error_Handler();  // Gestione errore durante la chiusura del file
        return res;
    }

    return res;  // 	Restituisce FR_OK se non ci sono stati errori
}




void write_to_sd (int risultato)
{
	FRESULT res;
    UINT bw;
	res = f_open(&file_out, F_OUT, FA_WRITE | FA_OPEN_APPEND);
	if(res != FR_OK)
	{
		 Error_Handler();
	}
	res = f_write(&file_out, &risultato, sizeof(risultato), &bw);
	if((res != FR_OK)||(bw == 0))
	{
		Error_Handler();
	}
	res = f_close(&file_out);
	if(res != FR_OK)
	{
		 Error_Handler();
	}

}


void write_report(char* string, int string_len)
{
	FRESULT res;
	UINT bw;
	res = f_open(&f_report, F_REPORT, FA_WRITE | FA_OPEN_APPEND);
	if(res != FR_OK)
	{
		Error_Handler();
	}
	res = f_write(&f_report, string, string_len, &bw);
	if((res != FR_OK)||(bw != string_len))
	{
		Error_Handler();
	}
	res = f_close(&f_report);
	if((res != FR_OK)||(bw == 0))
	{
		Error_Handler();
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
