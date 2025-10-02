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
#include "usbd_cdc_if.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <st7735.h>
#include "fonts.h"
#include "testimg.h"
#include <math.h>

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
ADC_HandleTypeDef hadc2;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
long counter = 0;
bool written = false;
volatile long adc_callback_counter = 0;
#define ADCBUFSIZE 1000
uint32_t adcbuf[ADCBUFSIZE];
bool adc_ready = false;

char log_filename[32];
uint32_t file_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI4_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM2_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_USART2_UART_Init(void);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // init display
  ST7735_Init();

  // clear display
  ST7735_FillScreen(0x0000);

  // adc calibration
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED, DISABLE);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED, DISABLE);

  // start timer
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // definitions for upper and lower threshold
  // (detector is triggered if ADC value hits upper threshold)
  // (all values are buffered until ADC value hits lower threshold)
  // (then the mean value of these values is calculated)
  uint16_t upper_thresh = 400;
  uint16_t lower_thresh = 300;

  // define allowed coincidence time difference
  uint16_t coincidence_window = 100;

  bool triggered = false;
  bool triggered2 = false;

  uint32_t sumval = 0;
  uint32_t conv_cnt = 0;
  uint16_t res_val = 0;

  uint32_t sumval2 = 0;
  uint32_t conv_cnt2 = 0;
  uint16_t res_val2 = 0;

  uint32_t c1_evt_cnt = 0;
  uint32_t c2_evt_cnt = 0;
  uint32_t coinc_evt_cnt = 0;

  uint32_t times1[100];
  uint8_t counter1 = 0;
  float rate1 = 0;

  uint32_t times2[100];
  uint8_t counter2 = 0;
  float rate2 = 0;

  uint32_t coinc_times[20];
  uint8_t coinc_count = 0;
  float coinc_rate = 0;

  uint32_t last_count1 = 0;
  uint32_t last_count2 = 0;

  // show status on display
  ST7735_WriteString(0, 10, "No events.", Font_7x10, 0xFFFF, 0x0000);
  ST7735_WriteString(0, 30, "No events.", Font_7x10, 0xFFFF, 0x0000);

  float vref = 3.3f; // reference voltage

  uint32_t microsecs = __HAL_TIM_GET_COUNTER(&htim2);
  uint32_t microsecs2 = __HAL_TIM_GET_COUNTER(&htim2);

  // find lowest numbered unused log file
  findNewFilename();

  // main loop
  while (1)
  {
          // ADC readout
	      char buf[40];
	      HAL_ADC_Start(&hadc1);
	      HAL_ADC_Start(&hadc2);
	      HAL_ADC_PollForConversion(&hadc1, 1);
	      HAL_ADC_PollForConversion(&hadc2, 1);
	      uint16_t val = HAL_ADC_GetValue(&hadc1);
	      uint16_t val2 = HAL_ADC_GetValue(&hadc2);

	      // process data from 1st ADC
	      if (val >= upper_thresh) {
	    	  triggered = true;
	    	  sumval = sumval + val;
	    	  conv_cnt++;
	    	  microsecs = __HAL_TIM_GET_COUNTER(&htim2);
	      } else if ((val < lower_thresh) && (triggered)) {
	    	  res_val = (uint16_t)round(sumval/conv_cnt);

	    	  float voltage = (res_val / 4095.0f) * vref;
	     	  sprintf(buf, "%lu | %.3f V   ", microsecs, voltage);
	          ST7735_WriteString(0, 10, buf, Font_7x10, 0xFFFF, 0x0000);

	          times1[counter1] = microsecs;
	          counter1++;

	          last_count1 = microsecs;
	          if (counter1 == 100) {
	        	  uint32_t timediff = times1[99]-times1[0];
	        	  float timediff_in_min = timediff / 60000000.0f;
	        	  rate1 = 100/timediff_in_min;

	        	  char buf[20];
	        	  sprintf(buf, "%.1f", rate1);
	        	  ST7735_WriteString(0, 70, buf, Font_7x10, 0xFFFF, 0x0000);

	        	  counter1 = 0;
	          }

	          char usbbuf[20];
	          uint8_t buflen = sprintf(usbbuf, "0,%lu,%.3f\n", microsecs, voltage);
	          logToSD(usbbuf, buflen);
	          CDC_Transmit_FS((uint8_t*)usbbuf, buflen);

	          c1_evt_cnt++;

	          if ((c1_evt_cnt % 100) == 0) {
	        	  findNewFilename();
	          }

	    	  sumval = 0;
	    	  conv_cnt = 0;
	     	  triggered = false;

		      sprintf(buf, "%lu", c1_evt_cnt);
		      ST7735_WriteString(0, 50, buf, Font_7x10, 0xFFFF, 0x0000);
	      }

	      // process data from 2nd ADC
	      if (val2 >= upper_thresh) {
	    	  triggered2 = true;
	      	  sumval2 = sumval2 + val2;
	      	  conv_cnt2++;
	    	  microsecs2 = __HAL_TIM_GET_COUNTER(&htim2);
	      } else if ((val2 < lower_thresh) && (triggered2)) {
	    	  res_val2 = (uint16_t)round(sumval2/conv_cnt2);

	    	  float voltage2 = (res_val2 / 4095.0f) * vref;
	     	  sprintf(buf, "%lu | %.3f V   ", microsecs2, voltage2);
	          ST7735_WriteString(0, 30, buf, Font_7x10, 0xFFFF, 0x0000);

	          times2[counter2] = microsecs2;
	          counter2++;

	          last_count2 = microsecs2;

	          if (counter2 == 100) {
	        	  uint32_t timediff = times2[99]-times2[0];
	          	  float timediff_in_min = timediff / 60000000.0f;
	          	  rate2 = 100/timediff_in_min;

	          	  char buf[20];
	          	  sprintf(buf, "%.1f", rate2);
	          	  ST7735_WriteString(60, 70, buf, Font_7x10, 0xFFFF, 0x0000);

	          	  counter2 = 0;
	          }

	          char usbbuf[20];
	          uint8_t buflen = sprintf(usbbuf, "1,%lu,%.3f\n", microsecs2, voltage2);
	          logToSD(usbbuf, buflen);
	          CDC_Transmit_FS((uint8_t*)usbbuf, buflen);

	          c2_evt_cnt++;
	    	  sumval2 = 0;
	    	  conv_cnt2 = 0;
	     	  triggered2 = false;

	     	  sprintf(buf, "%lu", c2_evt_cnt);
		      ST7735_WriteString(60, 50, buf, Font_7x10, 0xFFFF, 0x0000);
	      }


	      if ((last_count1 > 0) && (last_count2 > 0)) {
	    	  if (abs(last_count1-last_count2) < coincidence_window) {
	    		  coinc_evt_cnt++;

	    		  coinc_times[coinc_count] = last_count1;
				  coinc_count++;

	    		  sprintf(buf, "%lu", coinc_evt_cnt);
	    		  ST7735_WriteString(120, 50, buf, Font_7x10, 0xF800, 0x0000);

	    		  last_count1 = 0;
	    		  last_count2 = 0;
	    	  }

	    	  if (coinc_count == 20) {
	    		  uint32_t timediff = coinc_times[19]-coinc_times[0];
	    		  float timediff_in_min = timediff / 60000000.0f;
	    		  coinc_rate = 20/timediff_in_min;

	    		  char buf[20];
	    		  sprintf(buf, "%.1f", coinc_rate);
	    		  ST7735_WriteString(120, 70, buf, Font_7x10, 0xFFFF, 0x0000);

	    		  coinc_count = 0;
	    	  }
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
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
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_USART2;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 12;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_18;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hsd1.Init.ClockDiv = 8;
  if (HAL_SD_Init(&hsd1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  htim2.Init.Prescaler = 119;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_LED_Pin|LCD_WR_RS_Pin|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE3 LCD_LED_Pin LCD_CS_Pin LCD_WR_RS_Pin
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|LCD_LED_Pin|LCD_CS_Pin|LCD_WR_RS_Pin
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint32_t micros(void)
{
    return __HAL_TIM_GET_COUNTER(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*
    if(htim->Instance == TIM2)
    {
        HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_3);
    }
    */
}

void logToSD(char* data, uint8_t datalen) {
    char TxBuffer[250];
    FATFS FatFs;
    FIL Fil;
    FRESULT FR_Status;
    UINT WWC; // Write Word Counter
    uint8_t buflen;

    // SD Karte mounten
    FR_Status = f_mount(&FatFs, SDPath, 1);
    if (FR_Status == FR_OK)
    {
        // Datei zum Anhängen öffnen (erstellt, falls nicht vorhanden)
        FR_Status = f_open(&Fil, log_filename, FA_WRITE | FA_OPEN_ALWAYS);
        if(FR_Status == FR_OK) {
            // Cursor ans Ende der Datei setzen
            f_lseek(&Fil, f_size(&Fil));
            // Daten schreiben
            f_write(&Fil, data, datalen, &WWC);
            // Datei schließen
            f_close(&Fil);
            buflen = sprintf(TxBuffer, "Data written to SD\r\n");
            CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
        } else {
            buflen = sprintf(TxBuffer, "Error! Opening log.txt failed, Error Code: (%i)\r\n", FR_Status);
            CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
        }
    } else {
        buflen = sprintf(TxBuffer, "Error! Mounting SD card failed, Error Code: (%i)\r\n", FR_Status);
        CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
    }
}

int fileExists(char* filename) {
	FATFS FatFs;
	FRESULT FR_Status;

	// mount sd card
	FR_Status = f_mount(&FatFs, SDPath, 1);
	if (FR_Status == FR_OK) {
		FILINFO fno;
		FRESULT res;

		res = f_stat(filename, &fno);
		if (res == FR_OK) {
			return 1;
		} else if (res == FR_NO_FILE) {
			return 0;
		} else {
			return 2;
		}
	}
	return 0;
}

void findNewFilename() {
	sprintf(log_filename, "log_%lu.txt", file_counter);
	while ((fileExists(log_filename) == 1) && (file_counter < 100000)) {
		file_counter++;
		sprintf(log_filename, "log_%lu.txt", file_counter);
	}

	char usbbuf[40];
	uint8_t buflen = sprintf(usbbuf, "%s\n", log_filename);
    CDC_Transmit_FS((uint8_t*)usbbuf, buflen);
}

void SDIO_SDCard_Test(void)
{
  char TxBuffer[250];
  FATFS FatFs;
  FIL Fil;
  FRESULT FR_Status;
  FATFS *FS_Ptr;
  UINT RWC, WWC; // Read/Write Word Counter
  DWORD FreeClusters;
  uint32_t TotalSize, FreeSpace;
  char RW_Buffer[200];
  uint8_t buflen;
  do
  {
    //------------------[ Mount The SD Card ]--------------------
    FR_Status = f_mount(&FatFs, SDPath, 1);
    if (FR_Status != FR_OK)
    {
      buflen = sprintf(TxBuffer, "Error! While Mounting SD Card, Error Code: (%i)\r\n", FR_Status);
      CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
      break;
    }
    buflen = sprintf(TxBuffer, "SD Card Mounted Successfully! \r\n\n");
    CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
    //------------------[ Get & Print The SD Card Size & Free Space ]--------------------
    f_getfree("", &FreeClusters, &FS_Ptr);
    TotalSize = (uint32_t)((FS_Ptr->n_fatent - 2) * FS_Ptr->csize * 0.5);
    FreeSpace = (uint32_t)(FreeClusters * FS_Ptr->csize * 0.5);
    buflen = sprintf(TxBuffer, "Total SD Card Size: %lu Bytes\r\n", TotalSize);
    CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
    buflen = sprintf(TxBuffer, "Free SD Card Space: %lu Bytes\r\n\n", FreeSpace);
    CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
    //------------------[ Open A Text File For Write & Write Data ]--------------------
    //Open the file
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
    if(FR_Status != FR_OK)
    {
      buflen = sprintf(TxBuffer, "Error! While Creating/Opening A New Text File, Error Code: (%i)\r\n", FR_Status);
      CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
      break;
    }
    buflen = sprintf(TxBuffer, "Text File Created & Opened! Writing Data To The Text File..\r\n\n");
    CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
    // (1) Write Data To The Text File [ Using f_puts() Function ]
    f_puts("Hello! From STM32 To SD Card Over SDMMC, Using f_puts()\n", &Fil);
    // (2) Write Data To The Text File [ Using f_write() Function ]
    strcpy(RW_Buffer, "Hello! From STM32 To SD Card Over SDMMC, Using f_write()\r\n");
    f_write(&Fil, RW_Buffer, strlen(RW_Buffer), &WWC);
    // Close The File
    f_close(&Fil);
    //------------------[ Open A Text File For Read & Read Its Data ]--------------------
    // Open The File
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_READ);
    if(FR_Status != FR_OK)
    {
      buflen = sprintf(TxBuffer, "Error! While Opening (MyTextFile.txt) File For Read.. \r\n");
      CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
      break;
    }
    // (1) Read The Text File's Data [ Using f_gets() Function ]
    f_gets(RW_Buffer, sizeof(RW_Buffer), &Fil);
    buflen = sprintf(TxBuffer, "Data Read From (MyTextFile.txt) Using f_gets():%s", RW_Buffer);
    CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
    // (2) Read The Text File's Data [ Using f_read() Function ]
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    buflen = sprintf(TxBuffer, "Data Read From (MyTextFile.txt) Using f_read():%s", RW_Buffer);
    CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
    // Close The File
    f_close(&Fil);
    buflen = sprintf(TxBuffer, "File Closed! \r\n\n");
    CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
    //------------------[ Open An Existing Text File, Update Its Content, Read It Back ]--------------------
    // (1) Open The Existing File For Write (Update)
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_OPEN_EXISTING | FA_WRITE);
    FR_Status = f_lseek(&Fil, f_size(&Fil)); // Move The File Pointer To The EOF (End-Of-File)
    if(FR_Status != FR_OK)
    {
      buflen = sprintf(TxBuffer, "Error! While Opening (MyTextFile.txt) File For Update.. \r\n");
      CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
      break;
    }
    // (2) Write New Line of Text Data To The File
    FR_Status = f_puts("This New Line Was Added During File Update!\r\n", &Fil);
    f_close(&Fil);
    memset(RW_Buffer,'\0',sizeof(RW_Buffer)); // Clear The Buffer
    // (3) Read The Contents of The Text File After The Update
    FR_Status = f_open(&Fil, "MyTextFile.txt", FA_READ); // Open The File For Read
    f_read(&Fil, RW_Buffer, f_size(&Fil), &RWC);
    buflen = sprintf(TxBuffer, "Data Read From (MyTextFile.txt) After Update:\r\n%s", RW_Buffer);
    CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
    f_close(&Fil);
    //------------------[ Delete The Text File ]--------------------
    // Delete The File
    /*
    FR_Status = f_unlink(MyTextFile.txt);
    if (FR_Status != FR_OK){
        sprintf(TxBuffer, "Error! While Deleting The (MyTextFile.txt) File.. \r\n");
        USC_CDC_Print(TxBuffer);
    }
    */
  } while(0);
  //------------------[ Test Complete! Unmount The SD Card ]--------------------
  FR_Status = f_mount(NULL, "", 0);
  if (FR_Status != FR_OK)
  {
	  buflen = sprintf(TxBuffer, "\r\nError! While Un-mounting SD Card, Error Code: (%i)\r\n", FR_Status);
      CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
  } else{
	  buflen = sprintf(TxBuffer, "\r\nSD Card Un-mounted Successfully! \r\n");
      CDC_Transmit_FS((uint8_t*)TxBuffer, buflen);
  }
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
