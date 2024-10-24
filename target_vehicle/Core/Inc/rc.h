#ifndef RC_H
#define RC_H

#define rc_huart   &huart3       // 此处定义的遥控器USART为USART3
#define MAX_RPM    3000.0        // 单位 转/min
#define MAX_ChangeRPM     3      // 最快的衰减/增加速度
#define Reduction_Ratio   36.0   // 减速比
#define PI                3.1415926f
#include "stm32f4xx_hal.h"
#include "usart.h"

typedef struct
{
	struct
	{ 
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		unsigned char s1;
		unsigned char s2;
	}rc;
	
	struct 
	{
		unsigned short x;
		unsigned short y;
		unsigned short z;
		unsigned char press_l;
		unsigned char press_r;
	}mouse;
	
	struct
	{
		unsigned short v;
	}key;
}RC_Ctl_t;

typedef struct
{
	float A_Speed;
	float B_Speed;
	float C_Speed;
	float D_Speed;
	float gimbal_ChangeRPM;      // 单位 转每分钟
	float HubMotor_A_ChangeRPM;
	float HubMotor_B_ChangeRPM;
	float HubMotor_C_ChangeRPM;
}motortarget;

typedef struct
{
	uint8_t data_id;
	uint8_t sign_flag;
	int16_t value;
	int32_t gimbal_set;
	int32_t HubMotor_set;
	int32_t HF_distance;
}InformationChange;

extern int flag_find_begin;     // 开始找滑轨一端
extern int flag_find_HF;        // 开始找滑轨的中点
extern int flag_distence_init;  // 开始位置初始化
extern motortarget MotorTarget;
extern RC_Ctl_t RC_Ctl;   			// 声明遥控器数据结构体
extern uint8_t date_rx_buffer[6];
extern InformationChange In_Change;
void rc_Init(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);

#endif
