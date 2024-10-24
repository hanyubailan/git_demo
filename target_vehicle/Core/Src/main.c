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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc.h"
#include "motol_pid.h"
#include "drv_can.h"
#include "decode_can.h"
#include "Rand_Num.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
pid_type_def gimbal = {0};
pid_type_def HubMotor_A = {0};
pid_type_def HubMotor_B = {0};
pid_type_def HubMotor_C = {0};
float PID_gimbal[3] = { 0.6f, 0.1f, 0.1f };       
float PID_HubMotor[3] = { 0.6f, 0.1f, 0.1f };
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  rc_Init();
  CAN_Init(&hcan1);                     
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);      // ʹ���ж�
  PID_init(&gimbal, PID_DELTA, PID_gimbal, 15000, 1000);                 // �����������������
  PID_init(&HubMotor_A, PID_DELTA, PID_HubMotor, 15000, 1000);           // ��������ٶȲ��4m/s��ok       
	PID_init(&HubMotor_B, PID_DELTA, PID_HubMotor, 15000, 1000);
	PID_init(&HubMotor_C, PID_DELTA, PID_HubMotor, 15000, 1000);
  gimbal.set = 500;         // ��ʼ�ٶ�
	HubMotor_A.set = 1500;
	HubMotor_B.set = 1500;
	HubMotor_C.set = 1500;
	RandRPM = 0;               // ���ģʽ�ֵ����ʼ�ٶ�
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		switch ( RC_Ctl.rc.s2 )        // ң��������
//		{
//			case 1:                      // ģʽһ������+ң���������ٶȿ���
//        gimbal.fdb = motor1.RPM;
//        HubMotor_A.fdb = motor2.RPM;
//    		HubMotor_B.fdb = motor3.RPM;
//    		HubMotor_C.fdb = motor4.RPM;
//			  gimbal.set = MODE1_ChangeRPM(gimbal.set, MotorTarget.gimbal_ChangeRPM);
//			  HubMotor_A.set = MODE1_ChangeRPM(HubMotor_A.set, MotorTarget.HubMotor_A_ChangeRPM);
//			  HubMotor_B.set = MODE1_ChangeRPM(HubMotor_B.set, MotorTarget.HubMotor_B_ChangeRPM);
//			  HubMotor_C.set = MODE1_ChangeRPM(HubMotor_C.set, MotorTarget.HubMotor_C_ChangeRPM);
//        PID_calc(&gimbal,gimbal.fdb,gimbal.set);
//        PID_calc(&HubMotor_A, HubMotor_A.fdb, HubMotor_A.set);
//        PID_calc(&HubMotor_B, HubMotor_B.fdb, HubMotor_B.set);
//        PID_calc(&HubMotor_C, HubMotor_C.fdb, HubMotor_C.set);
//    		CAN_SendData_3508(gimbal.out, HubMotor_A.out, HubMotor_B.out, HubMotor_C.out);
//		    HAL_Delay(2);
//		  	break;
//			
//			case 2:                      // �����ģʽ(��̨���٣��ֵ�����ת��)
//				RandRPM = abs_ChangeNum_value(RandRPM);
//			  CAN_SendData_3508(RandRPM,0,0,0);
//		   	HAL_Delay(2);
//				break;
//			
//			case 3:                      // ֹͣ����
//				CAN_SendData_3508(0, 0, 0, 0);
//		    HAL_Delay(2);
//				break;
//			
//			default:
//			break;
//		};
    
		// Ӳ�������
		// �ٶȿ���
		gimbal.fdb = motor1.RPM;
    HubMotor_A.fdb = motor2.RPM;
	  gimbal.set = In_Change.gimbal_set;            // �����ֵ����ʼ�ٶ�Ϊ0
		HubMotor_A.set = In_Change.HubMotor_set;      // �����ֵ����ʼ�ٶ�Ϊ0
    PID_calc(&gimbal,gimbal.fdb,gimbal.set);
    PID_calc(&HubMotor_A, HubMotor_A.fdb, HubMotor_A.set);
 		CAN_SendData_3508(gimbal.out, HubMotor_A.out, 0, 0);
		
		if(flag_distence_init == 1)       // ��ʼ����λ��Ȼ�����������ƶ�
		{
			while(flag_find_begin == 1)     // ֱ���ҵ������һ��
			{
				CAN_SendData_3508(gimbal.out,500,0,0); // ֱ������һ��
				HAL_Delay(2);
			}
			
			motor2.total_distance = 0;
			while(flag_find_HF == 1)                    // ֱ���ҵ������һ��
			{
				CAN_SendData_3508(gimbal.out,500,0,0);             // һֱ�˶�ֱ�������м�
				if( motor2.total_distance > HF_track_length + In_Change.HF_distance)
				{
					flag_find_HF = 0;
				};
				HAL_Delay(2);
			}
			flag_distence_init = 0;
			HubMotor_A.out = -abs(HubMotor_A.out);
			CAN_SendData_3508(gimbal.out,HubMotor_A.out,0,0); //��ʼ��������
		}
		
		if(motor2.total_distance < HF_track_length - In_Change.HF_distance)
		{
			HubMotor_A.out = chang_sign(HubMotor_A.out);
		}
		if(motor2.total_distance < HF_track_length + In_Change.HF_distance)
		{
			HubMotor_A.out = chang_sign(HubMotor_A.out);
		}
		
		HAL_Delay(2);
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)    // �ᴥ���ظı���ת��
{
	switch(GPIO_Pin)
	{
		case GPIO_PIN_3:
			chang_sign(HubMotor_A.set);
		  flag_find_begin = 0;            // ��־�Ѿ����ﻬ���һ��
		break;
		
		case GPIO_PIN_5:
			chang_sign(HubMotor_A.set);
		  flag_find_begin = 0;            // ��־�Ѿ����ﻬ���һ��
		break;
		
		default:
		break;
	}
};
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
