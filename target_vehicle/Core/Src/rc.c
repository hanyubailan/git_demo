#include "rc.h"

motortarget MotorTarget = {0};
RC_Ctl_t RC_Ctl = {0}; 
InformationChange In_Change = {0};
uint8_t sbus_rx_buffer[18]; 		// ����ң������������
uint8_t data_rx_buffer[6]; 		  // ����Ӳ���建������
int flag_distence_init = 0;     // �������������Ƿ���Ҫ������ĳ�ʼ��
int flag_find_begin = 0;
int flag_find_HF = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) 
{
	if(UartHandle == &huart3)   //ң�ش���
	{
		RC_Ctl.rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff;          
	  RC_Ctl.rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;       
	  RC_Ctl.rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;          
	  RC_Ctl.rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;           
	  RC_Ctl.rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                           
	  RC_Ctl.rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);    

	  RC_Ctl.mouse.x = sbus_rx_buffer[6] | (sbus_rx_buffer[7] << 8);                    //!< Mouse X axis
	  RC_Ctl.mouse.y = sbus_rx_buffer[8] | (sbus_rx_buffer[9] << 8);                    //!< Mouse Y axis
	  RC_Ctl.mouse.z = sbus_rx_buffer[10] | (sbus_rx_buffer[11] << 8);                  //!< Mouse Z axis  
	  RC_Ctl.mouse.press_l = sbus_rx_buffer[12];                                        //!< Mouse Left Is Press     
	  RC_Ctl.mouse.press_r = sbus_rx_buffer[13];                                        //!< Mouse Right Is Press
	  RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);   			            //!< KeyBoard value

		if( RC_Ctl.rc.s2 == 1 )
			{
				MotorTarget.gimbal_ChangeRPM = ((RC_Ctl.rc.ch1-1024.0)/660.0)*MAX_ChangeRPM;
				MotorTarget.HubMotor_A_ChangeRPM = ((RC_Ctl.rc.ch3-1024.0)/660.0)*MAX_ChangeRPM;
				MotorTarget.HubMotor_B_ChangeRPM = ((RC_Ctl.rc.ch3-1024.0)/660.0)*MAX_ChangeRPM;
				MotorTarget.HubMotor_C_ChangeRPM = ((RC_Ctl.rc.ch3-1024.0)/660.0)*MAX_ChangeRPM;
	  	}
			
	  HAL_UART_Receive_DMA(rc_huart,sbus_rx_buffer,18);  //����ң��������Ϣ
	}
	else if(UartHandle == &huart1)            //����Ӳ���������
	{
		if (data_rx_buffer[0] == 0xFF || data_rx_buffer[5] == 0xFE)
		{
			In_Change.data_id = data_rx_buffer[1];
			In_Change.sign_flag = data_rx_buffer[2];
			In_Change.value = (data_rx_buffer[3] << 8) | data_rx_buffer[4]; // �߰�λ | �Ͱ�λ
			if (In_Change.sign_flag != 0x00)      // �������λΪ00��Ϊ������01��Ϊ����
			{
				In_Change.value = -In_Change.value;
			}
			
			switch (In_Change.data_id)      		  // ����ɶԵ����ʵ�ʼ���
			{
				case 01:                            // ��̨�ٶ�
					In_Change.gimbal_set = In_Change.value; 
				break;
				
				case 02:                            // �ֵ���ٶ�
					In_Change.HubMotor_set = In_Change.value; 
				break;
				
				case 03:                            // �������ߵľ���
					In_Change.HF_distance = In_Change.value; 
				  flag_distence_init = 1;                         // ��ʾ������Ҫ��ʼ��
				  flag_find_begin = 1;                            // ������Ҫ��һ��
          flag_find_HF = 1;                               // ������Ҫ���е�
				break;
				
				default:
				break;
			}
		}
		
		HAL_UART_Receive_DMA(&huart1,data_rx_buffer,sizeof(data_rx_buffer));
	}
	
};

void rc_Init()
{
	HAL_UART_Receive_DMA(rc_huart,sbus_rx_buffer,sizeof(sbus_rx_buffer));  //����ң��������Ϣ
	HAL_UART_Receive_DMA(&huart1,data_rx_buffer,sizeof(data_rx_buffer));   //����Ӳ�������Ϣ
};


