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
#include "chlorine_sensor.h"
#include "turb.h"
#include "tla20xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RS485_DIR_PORT   GPIOA
#define RS485_DIR_PIN    GPIO_PIN_1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
TLA20XX_Handle_t tla;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

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
uint8_t uart1_rx;
static uint8_t  mb1_rx_buf[64];
static uint16_t mb1_rx_index = 0;
static volatile uint8_t mb1_done = false;

static volatile uint8_t mb3_done = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
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
				HAL_UART_Transmit(&huart2,(uint8_t*)msg_ph,strlen(msg_ph),HAL_MAX_DELAY);

			}
			else if(strcmp(cmd_buffer,"cl")==0 ||strcmp(cmd_buffer,"CL")==0 )
			{
				chlorine_calculation();
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
  /* USER CODE BEGIN 2 */

  RS485_TX_Mode();
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
//	  do_request = true;
	  Command_Dispatcher();
//	  UART1_ChangeStopBits(UART_STOPBITS_1);
	   ph_running();
//	   HAL_UART_Transmit(&huart2, (uint8_t*)"\nevde brooo !!!!!\r\n", 30, 100);

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
