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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* Sensor headers */
#include "sensor_common.h"
#include "Do_sensor.h"
#include "ph_sensor.h"
#include "ph_sensor_France.h"
#include "chlorine_sensor.h"
#include "turb.h"
#include "tla20xx.h"
#include "conductivity.h"
#include "ph_calculation.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RS485_DIR_PORT   GPIOB
#define RS485_DIR_PIN    GPIO_PIN_13

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
TLA20XX_Handle_t tla;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

static char dbg_buf[128];

uint8_t uart2_rx_byte;
uint8_t uart2_rx_buffer[64];
uint8_t rx_index = 0;
uint8_t uart2_rx_byte;
static uint8_t  i = 0;

volatile bool do_request   = false;
volatile bool temp_request = false;
volatile bool turb_request = false;
volatile bool ph_france = false;
volatile bool ph_Local = false;
uint8_t uart1_rx;
static uint8_t  mb1_rx_buf[64];
static uint16_t mb1_rx_index = 0;
static volatile uint8_t mb1_done = false;
static volatile uint8_t mb3_done = false;
//conductivity
volatile uint16_t adc_buffer_A0[NUM_SAMPLES];  // store A0 samples
volatile uint16_t adc_buffer_A1[NUM_SAMPLES];  // store A1 samples
uint16_t adc_buffer_TEMP[NUM_SAMPLES];         //store temp values
volatile uint8_t sample_index = 0;
volatile uint8_t adc_busy = 0;
volatile uint8_t adc_done = 0;
char msg[64];
volatile uint8_t positive_cycle = 0;
extern char uart_buf[32];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void DO_SendRequest(void);
float DO_Read(void);
uint16_t Modbus_CalcCRC(uint8_t *buf, uint16_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void RS485_TX_Mode(void)
{
    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_SET);
}

// RX = LOW
static inline void RS485_RX_Mode(void)
{
    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, GPIO_PIN_RESET);
}
int Modbus_ReadRegisters_IT(UART_HandleTypeDef *huart,
                            uint8_t slave_id,
                            uint16_t reg_addr,
                            uint16_t num_regs,
                            uint16_t *out,
                            bool out_is_u16)
{
    uint8_t tx[8];
    uint16_t crc;
    uint16_t resp_len = 5 + num_regs * 2;

    /* Select Modbus context */
    uint8_t  *rx_buf;
    uint16_t *rx_index;
    volatile uint8_t *rx_done;

    if (huart == &huart1)          // DO Modbus
    {
        rx_buf   = mb1_rx_buf;
        rx_index = &mb1_rx_index;
        rx_done  = &mb1_done;
    }
    else
    {
        return -10; // invalid UART
    }

    /* Prepare request */
    tx[0] = slave_id;
    tx[1] = 0x03;
    tx[2] = reg_addr >> 8;
    tx[3] = reg_addr & 0xFF;
    tx[4] = num_regs >> 8;
    tx[5] = num_regs & 0xFF;

    crc = Modbus_CalcCRC(tx, 6);
    tx[6] = crc & 0xFF;
    tx[7] = crc >> 8;

    /* Send request */
    RS485_TX_Mode();
    HAL_UART_Transmit(huart, tx, 8, 100);
    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET);
    RS485_RX_Mode();

    /* Start RX interrupt */
    *rx_index = 0;
    *rx_done  = 0;
    HAL_UART_Receive_IT(huart, &rx_buf[0], 1);

    /* Wait for response */
    uint32_t start = HAL_GetTick();
    while (!(*rx_done))
    {
        if (HAL_GetTick() - start > 1000)
            return -2; // timeout
    }

    /* Validate frame */
    if (rx_buf[0] != slave_id || rx_buf[1] != 0x03)
        return -3;

    crc = Modbus_CalcCRC(rx_buf, resp_len - 2);
    uint16_t crc_rx = rx_buf[resp_len - 2] |
                      (rx_buf[resp_len - 1] << 8);
    if (crc != crc_rx)
        return -5;

    /* Copy data */
    if (out_is_u16)
    {
        for (uint16_t i = 0; i < num_regs; i++)
        {
            out[i] = ((uint16_t)rx_buf[3 + i*2] << 8) |
                      rx_buf[4 + i*2];
        }
    }
    else
    {
        memcpy(out, &rx_buf[3], num_regs * 2);
    }

    return 0;
}

uint16_t Modbus_CalcCRC(uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];
        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
void I2C2_Scan(void)
{
    char msg[100];

    for (uint8_t addr = 1; addr < 127; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c2, addr << 1, 2, 10) == HAL_OK)
        {
            sprintf(msg, "I2C2 device @ 0x%02X\r\n", addr);
            HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
        }
    }
}
void UART1_ChangeStopBits(uint32_t stopBits)
{
    /* 1. Stop any ongoing RX/TX */
    HAL_UART_Abort(&huart1);

    /* 2. De-initialize UART */
    HAL_UART_DeInit(&huart1);

    /* 3. Change only stop bits */
    huart1.Init.StopBits = stopBits;   // UART_STOPBITS_1 or UART_STOPBITS_2

    /* 4. Re-initialize UART */
    HAL_UART_Init(&huart1);
    HAL_Delay(5);   // ✅ REQUIRED for RS485 settle time
    /* 5. Restart RX in 1-byte interrupt mode */
    mb1_rx_index = 0;
    mb1_done = false;

    HAL_UART_Receive_IT(&huart1, &uart1_rx, 1);
}
void Command_Dispatcher(void)
{
    if (do_request)
    {
        do_request = false;
        UART1_ChangeStopBits(UART_STOPBITS_2);

        float do_val = DO_ReadConcentration_2pt();
        int len = snprintf(dbg_buf, sizeof(dbg_buf),
                           "\nDO = %.2f mg/L\r\n", do_val);
        HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buf, len, 100);
    }
    if (temp_request)
    {
        temp_request = false;
        UART1_ChangeStopBits(UART_STOPBITS_2);

        float t = DO_ReadTemperature();
        int len = snprintf(dbg_buf, sizeof(dbg_buf),
                           "\nTEMP = %.2f C\r\n", t);
        HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buf, len, 100);
    }
    if(turb_request)
    {
    	turb_request = false;
    	UART1_ChangeStopBits(UART_STOPBITS_1);
    	float ntu = TURB_ReadNTU();
		int len = snprintf(dbg_buf, sizeof(dbg_buf),"\r\nTURB = %.3f NTU\r\n", ntu);
		HAL_UART_Transmit(&huart2, (uint8_t*)dbg_buf, len, 50);
    }
    if(ph_france)
    {
    	ph_france = false;
    	uint16_t count = 0;

    	while(1)
    	{
    		ph_france_running();
    		count++;
    		if(count >= 101)
    		{
    			break;
    		}

    	}
//    	HAL_UART_Transmit(&huart2,
//    		                  (uint8_t*)uart_buf,
//    		                  strlen(uart_buf),
//    		                  HAL_MAX_DELAY);
    }
    if(ph_Local)
    {
    	ph_Local = false;
    	uint16_t count = 0;
    	while(1)
		{
			ph_running();
			count++;
			if(count >= 101)
			{
				break;
			}

		}

    }
}
//conductivity timer,adc
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // TIM2: Detect positive PWM half
    if(htim->Instance == TIM2)
    {
        positive_cycle ^= 1;  // toggle every 2.5 ms

        if(positive_cycle)    // only positive half
        {

            sample_index = 0;
            adc_done = 0;

            __HAL_TIM_SET_COUNTER(&htim6, 0);      // reset TIM6 counter
            HAL_TIM_Base_Start_IT(&htim6);         // start sampling timer
        }
    }

    // TIM6: trigger ADC samples
    if(htim->Instance == TIM6)
    {
        if(sample_index < NUM_SAMPLES  && !adc_busy)
        {
        	adc_busy = 1;             // mark ADC as busy
            HAL_ADC_Start_IT(&hadc1);  // trigger one ADC conversion
        }
        else
        {
            HAL_TIM_Base_Stop_IT(&htim6); // stop TIM6 after 15 samples
            adc_done = 1;                  // signal main loop
        }
    }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static uint8_t ch_index = 0;  // track which channel is being read

    if(hadc->Instance == ADC1 && sample_index < NUM_SAMPLES)
    {
        uint16_t val = HAL_ADC_GetValue(hadc);

        if(ch_index == 0)
            adc_buffer_A0[sample_index] = val;  // store A0
        else if(ch_index == 1)
            adc_buffer_A1[sample_index] = val;  // store A1
        else if(ch_index == 2)
        	 adc_buffer_TEMP[sample_index] = val; //store temp values

        ch_index++;

        if(ch_index >= 3)  // after both channels, move to next sample
        {
            ch_index = 0;
            sample_index++;
            adc_busy = 0;
        }
    }
}
/* ================= UART RX CALLBACK ================= */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2)
	{
		char cmd = (char)uart2_rx_byte;
		if(cmd != '\r')
		{
			uart2_rx_buffer[i] = cmd;
			i++;
		}
		else
		{
			uart2_rx_buffer[i] = '\0';  // NULL terminate the string
			char* cmd_buffer = (char*)uart2_rx_buffer;
			if(strcmp(cmd_buffer,"DO")==0 ||strcmp(cmd_buffer,"do")==0 )
			{
//				HAL_UART_Transmit(&huart2, (uint8_t*)"\Do comming !!!!!\r\n", 30, 100);
				 do_request = true;
			}
			else if(strcmp(cmd_buffer,"TEMP")==0 ||strcmp(cmd_buffer,"temp")==0 )
			{
				//HAL_UART_Transmit(&huart2, (uint8_t*)"\nTemperature successful\n\r", 25, 100);
				 temp_request = true;
			}
			else if(strcmp(cmd_buffer,"TURB")==0 ||strcmp(cmd_buffer,"turb")==0 )
			{
				turb_request = true;
			}
			else if(strcmp(cmd_buffer,"ph")==0 ||strcmp(cmd_buffer,"PH")==0 )
			{
				ph_Local = true;
//				HAL_UART_Transmit(&huart2,(uint8_t*)msg_ph,strlen(msg_ph),HAL_MAX_DELAY);

			}
			else if(strcmp(cmd_buffer,"ph fra")==0 ||strcmp(cmd_buffer,"PH FRA")==0 )
			{
				 ph_france = true;
//				HAL_UART_Transmit(&huart2,(uint8_t*)msg_ph_france,strlen(msg_ph_france),HAL_MAX_DELAY);

			}
			else if(strcmp(cmd_buffer,"cl")==0 ||strcmp(cmd_buffer,"CL")==0 )
			{
				chlorine_calculation();
			}
			else if(strcmp(cmd_buffer,"ec")==0 ||strcmp(cmd_buffer,"EC")==0 )
			{
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
				HAL_MAX_DELAY);
			}
			else
			{
				HAL_UART_Transmit(&huart2, (uint8_t*)"\nEnter a Valid Command !!!!!\r\n", 30, 100);

			}

			 i = 0;  // RESET index for next command!
			 memset(uart2_rx_buffer, 0, sizeof(uart2_rx_buffer));  // Clear buffer

		}
		HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1); //re enable the interrupt
	}
	else if (huart == &huart1)
	{
	    mb1_rx_buf[mb1_rx_index++] = uart1_rx;   // ✅ STORE FIRST

	    if (mb1_rx_index >= 5)
	    {
	        uint8_t expected = 5 + mb1_rx_buf[2];
	        if (mb1_rx_index >= expected)
	            mb1_done = true;
	    }

	    HAL_UART_Receive_IT(&huart1, &uart1_rx, 1);
	}
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
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  RS485_TX_Mode();
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
  HAL_UART_Receive_IT(&huart1, &uart1_rx, 1);
  TLA20XX_Init(&tla, &hi2c2, (0x49 << 1));
  I2C2_Scan();
  TLA20XX_Begin(&tla);
  ph_setup();
  chlorine_setup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  Command_Dispatcher();
	  Calculation_Conductivity();
	  ph_France_Runnnig_temp();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c2.Init.Timing = 0x00201D2B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim1.Init.Prescaler = 3199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 3199;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Init.StopBits = UART_STOPBITS_2;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

