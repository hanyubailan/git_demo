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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t Rxbuf[18] = {0};
struct
 {
 uint16_t ch0;
 uint16_t ch1;
 uint16_t ch2;
 uint16_t ch3;
 uint8_t s1;
 uint8_t s2;
 }dr16;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{//用于接收数据的函数，使用DMA方式来接收UART数据
  uint32_t tmp1 = 0;
  tmp1 = huart->RxState;
	//创建一个临时变量tmp1，并将UART的接收状态赋值给它
	if (tmp1 == HAL_UART_STATE_READY)//判断UART是否处于就绪状态
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}
 
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;
 
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
		//启动DMA接收，源地址为UART数据寄存器，目标地址为接收缓冲区
	
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
		//使能UART的DMA接收功能
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
};


void Dr16_Init()   //初始化
{

	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);        //使能空闲中断

	uart_receive_dma_no_it(&huart1, Rxbuf, 50);
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Rxbuf,18);    //开始DMA接收
};


void rc_callback_handler()
	{

		dr16.ch0 = ((int16_t)Rxbuf[0] | ((int16_t)Rxbuf[1] << 8)) & 0x07FF; 
    dr16.ch1 = (((int16_t)Rxbuf[1] >> 3) | ((int16_t)Rxbuf[2] << 5)) & 0x07FF;
    dr16.ch2 = (((int16_t)Rxbuf[2] >> 6) | ((int16_t)Rxbuf[3] << 2) |((int16_t)Rxbuf[4] << 10)) & 0x07FF;
    dr16.ch3 = (((int16_t)Rxbuf[4] >> 1) | ((int16_t)Rxbuf[5]<<7)) & 0x07FF;
    dr16.s1 = ((Rxbuf[5] >> 4) & 0x000C) >> 2;
    dr16.s2 = ((Rxbuf[5] >> 4) & 0x0003);
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Rxbuf,18);   //重新开启（noemal和circal？？？）
	
};


static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	//清除UART的空闲标志，以便下一次接收时能够正确检测到空闲状态
	

		__HAL_DMA_DISABLE(huart->hdmarx);//失能DMA接收，防止下一次接收的数据在上一次数据的尾部，而不是全新的数据
 
    rc_callback_handler( );	//处理接收的数据并解码
		
		__HAL_DMA_SET_COUNTER(huart->hdmarx, 50);//设置DMA接收预定义的缓冲区的长度，以便为下一次接收做好准备
		__HAL_DMA_ENABLE(huart->hdmarx);//重新启用DMA接收，以便继续接收数据
	

};

void uart_receive_handler(UART_HandleTypeDef *huart)
{//用于检查UART接收状态并在接收到空闲状态时调用相应的回调函数
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && //检查UART是否设置了空闲标志，表示UART接收完成并进入空闲状态
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))//检查UART空闲中断是否被使能，只有在中断使能的情况下，才会处理空闲状态
	{
		uart_rx_idle_callback(huart);//调用之前定义的函数，处理接收到的数据
	}
};

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
//rotor_speed  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  USART_Init(&huart1,my_decode_func);
	Dr16_Init();
	
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
