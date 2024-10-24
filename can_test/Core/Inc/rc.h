#ifndef RC_H
#define RC_H

#define rc_huart &huart3         // �˴������ң����USARTΪUSART3
#define chassis_HW 0.3           // ���̰�� ��λ m
#define chassis_LH 0.38          // ���̰볤 ��λ m
#define MAX_RPM    1500.0         //          ��λ ת/min
#define MAX_AngleSpeed    0.5    // �����ٶ� ��λ rad/s
#define Reduction_Ratio   36.0   // ���ٱ�
#define R                 0.07   // ���ְ뾶  ��λ m
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
	float A_RPM;      // ��λ תÿ����
	float B_RPM;
	float C_RPM;
	float D_RPM;
	float Vx;
	float Vy;
	float Vz;
}motortarget;

extern motortarget MotorTarget;
extern RC_Ctl_t RC_Ctl;   			// ����ң�������ݽṹ��

void Drive_Motor(void);
void rc_Init(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);

#endif
