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
{//���ڽ������ݵĺ�����ʹ��DMA��ʽ������UART����
  uint32_t tmp1 = 0;
  tmp1 = huart->RxState;
	//����һ����ʱ����tmp1������UART�Ľ���״̬��ֵ����
	if (tmp1 == HAL_UART_STATE_READY)//�ж�UART�Ƿ��ھ���״̬
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}
 
		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;
 
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
		//����DMA���գ�Դ��ַΪUART���ݼĴ�����Ŀ���ַΪ���ջ�����
	
		SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
		//ʹ��UART��DMA���չ���
		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
};


void Dr16_Init()   //��ʼ��
{

	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);        //ʹ�ܿ����ж�

	uart_receive_dma_no_it(&huart1, Rxbuf, 50);
//	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Rxbuf,18);    //��ʼDMA����
};


void rc_callback_handler()
	{

		dr16.ch0 = ((int16_t)Rxbuf[0] | ((int16_t)Rxbuf[1] << 8)) & 0x07FF; 
    dr16.ch1 = (((int16_t)Rxbuf[1] >> 3) | ((int16_t)Rxbuf[2] << 5)) & 0x07FF;
    dr16.ch2 = (((int16_t)Rxbuf[2] >> 6) | ((int16_t)Rxbuf[3] << 2) |((int16_t)Rxbuf[4] << 10)) & 0x07FF;
    dr16.ch3 = (((int16_t)Rxbuf[4] >> 1) | ((int16_t)Rxbuf[5]<<7)) & 0x07FF;
    dr16.s1 = ((Rxbuf[5] >> 4) & 0x000C) >> 2;
    dr16.s2 = ((Rxbuf[5] >> 4) & 0x0003);
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Rxbuf,18);   //���¿�����noemal��circal��������
	
};


static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	//���UART�Ŀ��б�־���Ա���һ�ν���ʱ�ܹ���ȷ��⵽����״̬
	

		__HAL_DMA_DISABLE(huart->hdmarx);//ʧ��DMA���գ���ֹ��һ�ν��յ���������һ�����ݵ�β����������ȫ�µ�����
 
    rc_callback_handler( );	//������յ����ݲ�����
		
		__HAL_DMA_SET_COUNTER(huart->hdmarx, 50);//����DMA����Ԥ����Ļ������ĳ��ȣ��Ա�Ϊ��һ�ν�������׼��
		__HAL_DMA_ENABLE(huart->hdmarx);//��������DMA���գ��Ա������������
	

};

void uart_receive_handler(UART_HandleTypeDef *huart)
{//���ڼ��UART����״̬���ڽ��յ�����״̬ʱ������Ӧ�Ļص�����
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && //���UART�Ƿ������˿��б�־����ʾUART������ɲ��������״̬
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))//���UART�����ж��Ƿ�ʹ�ܣ�ֻ�����ж�ʹ�ܵ�����£��Żᴦ�����״̬
	{
		uart_rx_idle_callback(huart);//����֮ǰ����ĺ�����������յ�������
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
