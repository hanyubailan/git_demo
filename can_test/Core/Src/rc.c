#include "rc.h"

motortarget MotorTarget = {0};
RC_Ctl_t RC_Ctl = {0}; 
uint8_t sbus_rx_buffer[18]; 		// 声明遥控器缓存数组


void Drive_Motor(void)          // 计算小车的x,y,z速度与各轮的速度的关系
{
	MotorTarget.A_Speed = (MotorTarget.Vx+MotorTarget.Vy-MotorTarget.Vz*(chassis_HW+chassis_LH));
	MotorTarget.B_Speed = (MotorTarget.Vx-MotorTarget.Vy-MotorTarget.Vz*(chassis_HW+chassis_LH));
	MotorTarget.C_Speed = (MotorTarget.Vx+MotorTarget.Vy+MotorTarget.Vz*(chassis_HW+chassis_LH));
	MotorTarget.D_Speed = (MotorTarget.Vx-MotorTarget.Vy+MotorTarget.Vz*(chassis_HW+chassis_LH));
	MotorTarget.A_RPM = (MotorTarget.A_Speed*60.0*Reduction_Ratio)/(2.0*PI*R);
	MotorTarget.B_RPM = (MotorTarget.B_Speed*60.0*Reduction_Ratio)/(2.0*PI*R);
	MotorTarget.C_RPM = (MotorTarget.C_Speed*60.0*Reduction_Ratio)/(2.0*PI*R);
	MotorTarget.D_RPM = (MotorTarget.D_Speed*60.0*Reduction_Ratio)/(2.0*PI*R);
};

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) 
{
	if(UartHandle == rc_huart)   //遥控处理
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
	  RC_Ctl.key.v = sbus_rx_buffer[14] | (sbus_rx_buffer[15] << 8);   			//!< KeyBoard value

//	      p_out = (RC_Ctl.rc.ch3 - 1024.0) / 660.0;     //3508电机参数
//	      motor_pid_angle.set	= (RC_Ctl.rc.ch3 -1024.0 ) / 660.0 *  8191.0;
		
//    MotorTarget.Vx = (((RC_Ctl.rc.ch1 - 1024.0) / 660.0)*MAX_RPM*2*PI*R)/(60.0*Reduction_Ratio); 
//    MotorTarget.Vy = (((RC_Ctl.rc.ch0 - 1024.0) / 660.0)*MAX_RPM*2*PI*R)/(60.0*Reduction_Ratio); 
//    MotorTarget.Vz = ((RC_Ctl.rc.ch2 - 1024.0) / 660.0)*MAX_AngleSpeed; 
		
//		Drive_Motor();
		
	  HAL_UART_Receive_DMA(&huart3,sbus_rx_buffer,18);  //接收遥控器的信息
	};
	
};

void rc_Init()
{
	HAL_UART_Receive_DMA(rc_huart,sbus_rx_buffer,18);  //接收遥控器的信息
};


