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
#include "motol_pid.h"
#include "drv_can.h"
#include "rc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//float abs_max_speed = 4000.0f;  // ���ת��   
//float p_out;                    // ������ֵ��ռ�ȣ���������
//float actual_speed = 0.0f;     // ʵ�ʲ�����ת�٣�����ͨ���������������������Ȼ�ȡ��
//float pid_output = 0.0f;       // PID ���ֵ�����ڿ��Ƶ��

pid_type_def chassis_A = {0};
pid_type_def chassis_B = {0};
pid_type_def chassis_C = {0};
pid_type_def chassis_D = {0};
float PID_chassis[3] = { 0.6f, 0.1f, 0.1f };
//pid_type_def motor_pid_speed;
//float PID_angle[3] = { 0.08f, 0.03f, 0.04f };
//float PID_speed[3] = { 0.08f, 0.03f, 0.04f };

typedef struct {
    int16_t rotor_angle;      // ת�ӻ�е�Ƕ�
    int16_t RPM;      // ת��ת��
    int16_t torque_current;   // ʵ��ת�ص���
    uint8_t motor_temperature; // ����¶�
} CanData;                     // ���յ����Ϣ


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

// ����can
void CAN_SendData_3508(int16_t motorl,int16_t motor2,int16_t motor3,int16_t motor4)
{    
	CAN_TxHeaderTypeDef hcan1_tx;
	uint8_t DATA[8];	
	uint32_t mailbox;
	
	hcan1_tx.DLC = 0x08;               //date_CAN1����ÿһ���ĳ���
	hcan1_tx.IDE = CAN_ID_STD;         //�����ñ�׼(11λ)������չID(29λ)�ı�ʶ������֡����
//  hcan1_tx.ExtId =  ;                //��������ı�ʶ��  
	hcan1_tx.StdId = 0x200;            //��ʶ�������˵��������
	hcan1_tx.RTR = CAN_RTR_DATA;       //CAN_RTR_DATA��ʾ����֡��ʵ�ʴ������ݣ�CAN_RTR_REMOTEԶ��֡������Զ�̽ڵ㷢������
//	hcan1_tx.TransmitGlobalTime =   ;  //����ָʾ�Ƿ��ڷ��͵���Ϣ�а���ȫ��ʱ�����������CAN FD�У����������Ϊ1����������֡��ĩβ���һ��ʱ�������ʾ֡���͵�ʱ�䡣

	DATA[0] = motorl >> 8;
	DATA[1] = motorl & 0xFF;
	DATA[2] = motor2 >> 8;
	DATA[3] = motor2 & 0xFF;
	DATA[4] = motor3 >> 8;
	DATA[5] = motor3 & 0xFF;
	DATA[6] = motor4 >> 8;
	DATA[7] = motor4 & 0xFF;
	HAL_CAN_AddTxMessage(&hcan1,&hcan1_tx,DATA,&mailbox);	//�˺���������Ӷ����ȥ�����������������ȼ�����HAL_CAN_Transmitֱ�ӷ��͵�CAN������
}

void CAN_SendData_6020(int16_t motorl,int16_t motor2,int16_t motor3,int16_t motor4)
{    
	CAN_TxHeaderTypeDef hcan1_tx;
	uint8_t DATA[8];	
	uint32_t mailbox;
	
	hcan1_tx.DLC = 0x08;               //date_CAN1����ÿһ���ĳ���
	hcan1_tx.IDE = CAN_ID_STD;         //�����ñ�׼(11λ)������չID(29λ)�ı�ʶ������֡����  
	hcan1_tx.StdId = 0X1FF;            //��ʶ�������˵��������
	hcan1_tx.RTR = CAN_RTR_DATA;       //CAN_RTR_DATA��ʾ����֡��ʵ�ʴ������ݣ�CAN_RTR_REMOTEԶ��֡������Զ�̽ڵ㷢������

	DATA[0] = motorl >> 8;
	DATA[1] = motorl & 0xFF;
	DATA[2] = motor2 >> 8;
	DATA[3] = motor2 & 0xFF;
	DATA[4] = motor3 >> 8;
	DATA[5] = motor3 & 0xFF;
	DATA[6] = motor4 >> 8;
	DATA[7] = motor4 & 0xFF;
	
	HAL_CAN_AddTxMessage(&hcan1,&hcan1_tx,DATA,&mailbox);   //�˺���������Ӷ����ȥ�����������������ȼ�����HAL_CAN_Transmitֱ�ӷ��͵�CAN������
}

CanData decoded_data;   //��������ĵ����Ϣ
CanData motor1; 
CanData motor2; 
CanData motor3; 
CanData motor4; 

//���������ص���ĺ���
CanData decode_can_data(uint8_t data[8]) {
    // ����ת�ӻ�е�Ƕ�
    decoded_data.rotor_angle = ((uint16_t)data[0] << 8) | data[1];
    
    // ����ת��ת��
    decoded_data.RPM = ((int16_t)data[2] << 8) | data[3];
    
    // ����ʵ��ת�ص���
    decoded_data.torque_current = ((int16_t)data[4] << 8) | data[5];
    
    // �������¶�
    decoded_data.motor_temperature = data[6];
    
    // ���ؽ����Ľṹ��
    return decoded_data;
}

uint8_t can_rx_data[32];  // ���ڴ洢���յ�������
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{  CAN_RxHeaderTypeDef rxHeader;  // ���ڴ洢����ͷ��Ϣ
  
  // ȷ�� CAN1 ��������
  if (hcan->Instance == CAN1)
  {
    // ��ȡ���յ�������
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, can_rx_data) == HAL_OK)
    {
      switch (rxHeader.StdId)
			{
				case 0x201:
					motor1 = decode_can_data(can_rx_data);
				  break;
				
				case 0x202:
					motor2 = decode_can_data(can_rx_data);
				  break;
				
				case 0x203:
					motor3 = decode_can_data(can_rx_data);
				  break;
				
				case 0x204:
					motor4 = decode_can_data(can_rx_data);
				  break;
				
				default:
					break;
			}
    }
  }
}

/* �Ƕ�Pidʱ���ڸ���tar��cur֮������ŵ���, �������ٽ���PID����*/
void Handle_Angle8191_PID_Over_Zero(int32_t *tar, int16_t *cur)
{
	if(*tar - *cur > 4096)    //4096 ����Ȧ��е�Ƕ�
	{
		*cur += 8192;        //8191,8192����ν�ˣ���������
	}
	else if(*tar - *cur < -4096)
	{
		*cur = *cur - 8192;
	}
}

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
  /* USER CODE BEGIN 2 */
	rc_Init();
  CAN_Init(&hcan1);                     
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);      //ʹ���ж�
  PID_init(&chassis_A, PID_DELTA, PID_chassis, 15000, 1000);             //16384
  PID_init(&chassis_B, PID_DELTA, PID_chassis, 15000, 1000);
	PID_init(&chassis_C, PID_DELTA, PID_chassis, 15000, 1000);
	PID_init(&chassis_D, PID_DELTA, PID_chassis, 15000, 1000);
	
//	// ��ʼ�� PID ����������
//  // Ŀ�꣺pid�����������������
//  PID_init(&motor_pid_angle, PID_DELTA, PID_angle, 3000, 200);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// ���ֵ��̽���
//    chassis_A.fdb = motor3.RPM;
//    chassis_B.fdb = motor1.RPM;
//		chassis_C.fdb = motor2.RPM;
//		chassis_D.fdb = motor4.RPM;
//		chassis_A.set = MotorTarget.A_RPM;
//		chassis_B.set = MotorTarget.B_RPM;
//		chassis_C.set = MotorTarget.C_RPM;
//		chassis_D.set = MotorTarget.D_RPM;
//    PID_calc(&chassis_A,chassis_A.fdb,chassis_A.set);   
//    PID_calc(&chassis_B,chassis_B.fdb,chassis_B.set);
//    PID_calc(&chassis_C,chassis_C.fdb,chassis_C.set);
//    PID_calc(&chassis_D,chassis_D.fdb,chassis_D.set);
//  	CAN_SendData_3508(chassis_B.out, chassis_C.out, chassis_A.out, chassis_D.out);
		
//		//6020���˫������
//		Handle_Angle8191_PID_Over_Zero( &motor_pid_angle.set, &decoded_data.rotor_angle);
//		
//    motor_pid_angle.fdb = decoded_data.rotor_angle;
//    PID_calc( &motor_pid_angle, motor_pid_angle.fdb, motor_pid_angle.set);
//		
//		motor_pid_speed.set = motor_pid_angle.out;
//		motor_pid_speed.fdb = decoded_data.rotor_speed;
//		PID_calc( &motor_pid_speed, motor_pid_speed.fdb, motor_pid_speed.set);
//		
//		CAN_SendData_6020( motor_pid_speed.out );


//		HAL_Delay(2);

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
