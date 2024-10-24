#include "rc.h"

motortarget MotorTarget = {0};
RC_Ctl_t RC_Ctl = {0}; 
InformationChange In_Change = {0};
uint8_t sbus_rx_buffer[18]; 		// 声明遥控器缓存数组
uint8_t data_rx_buffer[6]; 		  // 声明硬件板缓存数组
int flag_distence_init = 0;     // 用来区分现在是否需要定距离的初始化
int flag_find_begin = 0;
int flag_find_HF = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) 
{
	if(UartHandle == &huart3)   //遥控处理
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
			
	  HAL_UART_Receive_DMA(rc_huart,sbus_rx_buffer,18);  //接收遥控器的信息
	}
	else if(UartHandle == &huart1)            //接收硬件板的数据
	{
		if (data_rx_buffer[0] == 0xFF || data_rx_buffer[5] == 0xFE)
		{
			In_Change.data_id = data_rx_buffer[1];
			In_Change.sign_flag = data_rx_buffer[2];
			In_Change.value = (data_rx_buffer[3] << 8) | data_rx_buffer[4]; // 高八位 | 低八位
			if (In_Change.sign_flag != 0x00)      // 如果符号位为00则为负数，01则为正数
			{
				In_Change.value = -In_Change.value;
			}
			
			switch (In_Change.data_id)      		  // 待完成对电机的实际计算
			{
				case 01:                            // 云台速度
					In_Change.gimbal_set = In_Change.value; 
				break;
				
				case 02:                            // 轮电机速度
					In_Change.HubMotor_set = In_Change.value; 
				break;
				
				case 03:                            // 来回行走的距离
					In_Change.HF_distance = In_Change.value; 
				  flag_distence_init = 1;                         // 标示现在需要初始化
				  flag_find_begin = 1;                            // 现在需要找一端
          flag_find_HF = 1;                               // 现在需要找中点
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
	HAL_UART_Receive_DMA(rc_huart,sbus_rx_buffer,sizeof(sbus_rx_buffer));  //接收遥控器的信息
	HAL_UART_Receive_DMA(&huart1,data_rx_buffer,sizeof(data_rx_buffer));   //接收硬件板的信息
};


