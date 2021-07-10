/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Macro for count of samples to be displayed */
#define SAMPLE_COUNT  UINT16_C(30)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
#pragma location=0x30040200
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffers */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */
__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE]; /* Ethernet Receive Buffer */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE] __attribute__((section(".RxArraySection"))); /* Ethernet Receive Buffers */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//static const uint8_t BME688_ADDR = 0x76;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void bme68x_check_rslt(const char api_name[], int8_t rslt);

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf);
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,void *intf_ptr);
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,void *intf_ptr);

void bme68x_delay_us(uint32_t period, void *intf_ptr);
void SensorError_Handler(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
	uint8_t buf[64];
	char msg[64];

    switch (rslt)
    {
        case BME68X_OK:
        	sprintf(msg, "API name [%s]  Error [%d] : Sensor Init OK.\r\n", api_name, rslt);
        	strcpy((char*)buf, msg);
        	HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), HAL_MAX_DELAY);
            /* Do nothing */
            break;
        case BME68X_E_NULL_PTR:
        	sprintf(msg, "API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
			strcpy((char*)buf, msg);
			HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), HAL_MAX_DELAY);
			SensorError_Handler();
            break;
        case BME68X_E_COM_FAIL:
            sprintf(msg,"API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
            strcpy((char*)buf, msg);
			HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), HAL_MAX_DELAY);
			SensorError_Handler();
            break;
        case BME68X_E_INVALID_LENGTH:
            sprintf(msg,"API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
            strcpy((char*)buf, msg);
			HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), HAL_MAX_DELAY);
			SensorError_Handler();
            break;
        case BME68X_E_DEV_NOT_FOUND:
            sprintf(msg,"API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
            strcpy((char*)buf, msg);
			HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), HAL_MAX_DELAY);
			SensorError_Handler();
            break;
        case BME68X_E_SELF_TEST:
            sprintf(msg,"API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
            strcpy((char*)buf, msg);
			HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), HAL_MAX_DELAY);
			SensorError_Handler();
            break;
        case BME68X_W_NO_NEW_DATA:
            sprintf(msg,"API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
            strcpy((char*)buf, msg);
			HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), HAL_MAX_DELAY);
			SensorError_Handler();
            break;
        default:
            sprintf(msg,"API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
            strcpy((char*)buf, msg);
			HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), HAL_MAX_DELAY);
			SensorError_Handler();
            break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
	int8_t rslt = BME68X_OK;
	uint8_t dev_addr;

	if(bme != NULL) {
		/* Bus configuration : I2C */
		if (intf == BME68X_I2C_INTF)
		{
			dev_addr = BME68X_I2C_ADDR_LOW;
			bme->read = bme68x_i2c_read;
			bme->write = bme68x_i2c_write;
			bme->intf = BME68X_I2C_INTF;
		}

		bme->delay_us = bme68x_delay_us;
		bme->intf_ptr = &dev_addr;
		bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
	} else {
		rslt = BME68X_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * I2C read function map to STM32
 */
BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	HAL_StatusTypeDef status;
	uint8_t array[32] = { 0 };
	uint8_t stringpos = 0;
	array[0] = reg_addr;

	uint8_t dev_addr = 0x76 << 1; // Use 8-bit address

	while (HAL_I2C_IsDeviceReady(&hi2c1, dev_addr, 3, 100)!= HAL_OK)
	{

	}


	status = HAL_I2C_Mem_Read(&hi2c1,	    // I2C handle
				dev_addr,					// I2C address, left aligned
				(uint8_t) reg_addr,			// register address
				I2C_MEMADD_SIZE_8BIT,	    // BME688 uses 8bit register addresses
				(uint8_t*) (&array),		// write returned data to this variable
				len,						// how many bytes to expect returned
				100);					    // timeout

    if(status == HAL_OK) {
    	for (stringpos = 0; stringpos < len; stringpos++) {
    			*(reg_data + stringpos) = array[stringpos];
    		}
        return status;
	} else {
		SensorError_Handler();
		//return HAL_ERROR;
	}
}

/*!
 * I2C write function map to STM32
 */
BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
	HAL_StatusTypeDef status;
	uint8_t dev_addr = 0x76 << 1; // Use 8-bit address

	while (HAL_I2C_IsDeviceReady(&hi2c1, dev_addr, 3, 100)!= HAL_OK)
	{

	}

    status = HAL_I2C_Mem_Write(&hi2c1,	    // I2C handle
    			dev_addr,		            // I2C address, left aligned
    			(uint8_t) reg_addr,			// register address
    			I2C_MEMADD_SIZE_8BIT,	    // BME688 uses 8bit register addresses
    			(uint8_t*) (&reg_data),		// write returned data to reg_data
    			len,			        	// write how many bytes
    			100);					    // timeout

    if(status == HAL_OK) {
    	return status;
    } else {
    	SensorError_Handler();
    	//return HAL_ERROR;
    }
}

void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    HAL_Delay(period);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  struct bme68x_dev bme ={0};
  struct bme68x_conf conf ={0};
  struct bme68x_heatr_conf heatr_conf = {0};
  struct bme68x_data data = {0};
  uint8_t rslt;
  uint32_t del_period;
  uint32_t time_ms = 0;
  uint8_t n_fields;
  uint16_t sample_count = 1;
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  rslt = bme68x_interface_init(&bme, BME68X_I2C_INTF);
  bme68x_check_rslt("bme68x_interface_init", rslt);

  rslt = bme68x_init(&bme);
  bme68x_check_rslt("bme68x_init", rslt);

  conf.filter = BME68X_FILTER_OFF;
  conf.odr = BME68X_ODR_20_MS;//BME68X_ODR_NONE;
  conf.os_hum = BME68X_OS_16X;
  conf.os_pres = BME68X_OS_1X;
  conf.os_temp = BME68X_OS_2X;
  rslt = bme68x_set_conf(&conf, &bme);
  bme68x_check_rslt("bme68x_set_conf", rslt);

  heatr_conf.enable = BME68X_ENABLE;
  heatr_conf.heatr_temp = 300;
  heatr_conf.heatr_dur = 100;
  rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
  bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

  while (sample_count <= SAMPLE_COUNT)
      {
          rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
          bme68x_check_rslt("bme68x_set_op_mode", rslt);

          /* Calculate delay period in microseconds */
          del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
          bme.delay_us(del_period, bme.intf_ptr);

          //time_ms = coines_get_millis();

          /* Check if rslt == BME68X_OK, report or handle if otherwise */
          rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
          bme68x_check_rslt("bme68x_get_data", rslt);

          if (n_fields)
          {
  #ifdef BME68X_USE_FPU
              printf("%u, %lu, %.2f, %.2f, %.2f, %.2f, 0x%x\n",
                     sample_count,
                     (long unsigned int)time_ms,
                     data.temperature,
                     data.pressure,
                     data.humidity,
                     data.gas_resistance,
                     data.status);
  #else
              printf("%u, %lu, %d, %lu, %lu, %lu, 0x%x\n",
                     sample_count,
                     (long unsigned int)time_ms,
                     (data.temperature / 100),
                     (long unsigned int)data.pressure,
                     (long unsigned int)(data.humidity / 1000),
                     (long unsigned int)data.gas_resistance,
                     data.status);
  #endif
              sample_count++;
          }
      }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
	  HAL_Delay(500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 275;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
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
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  heth.Init.MACAddr[0] =   0x00;
  heth.Init.MACAddr[1] =   0x80;
  heth.Init.MACAddr[2] =   0xE1;
  heth.Init.MACAddr[3] =   0x00;
  heth.Init.MACAddr[4] =   0x00;
  heth.Init.MACAddr[5] =   0x00;
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c1.Init.Timing = 0x60404E72;
  hi2c1.Init.OwnAddress1 = 0x76 << 1;
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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_YELLOW_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  This function is executed in case of BME688 sensor error occurrence.
  * @retval None
  */
void SensorError_Handler(void)
{
  uint8_t buf[64];
  /* User can add his own implementation to report the HAL error return state */
  strcpy((char*)buf, "BME688 Sensor Error Occurred.\r\n");
  HAL_UART_Transmit(&huart3, buf, strlen((char*)buf), HAL_MAX_DELAY);
  while (1)
  {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
