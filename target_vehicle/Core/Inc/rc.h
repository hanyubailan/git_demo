#ifndef RC_H
#define RC_H

#define rc_huart   &huart3       // �˴������ң����USARTΪUSART3
#define MAX_RPM    3000.0        // ��λ ת/min
#define MAX_ChangeRPM     3      // ����˥��/�����ٶ�
#define Reduction_Ratio   36.0   // ���ٱ�
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
	float gimbal_ChangeRPM;      // ��λ תÿ����
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

extern int flag_find_begin;     // ��ʼ�һ���һ��
extern int flag_find_HF;        // ��ʼ�һ�����е�
extern int flag_distence_init;  // ��ʼλ�ó�ʼ��
extern motortarget MotorTarget;
extern RC_Ctl_t RC_Ctl;   			// ����ң�������ݽṹ��
extern uint8_t date_rx_buffer[6];
extern InformationChange In_Change;
void rc_Init(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);

#endif
