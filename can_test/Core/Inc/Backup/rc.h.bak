#ifndef RC_H
#define RC_H

#define rc_huart &huart3         // 此处定义的遥控器USART为USART3
#define chassis_HW 0.3           // 底盘半宽 单位 m
#define chassis_LH 0.38          // 底盘半长 单位 m
#define MAX_RPM    1500.0         //          单位 转/min
#define MAX_AngleSpeed    0.5    // 最大角速度 单位 rad/s
#define Reduction_Ratio   36.0   // 减速比
#define R                 0.07   // 麦轮半径  单位 m
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
	float A_RPM;      // 单位 转每分钟
	float B_RPM;
	float C_RPM;
	float D_RPM;
	float Vx;
	float Vy;
	float Vz;
}motortarget;

extern motortarget MotorTarget;
extern RC_Ctl_t RC_Ctl;   			// 声明遥控器数据结构体

void Drive_Motor(void);
void rc_Init(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);

#endif
