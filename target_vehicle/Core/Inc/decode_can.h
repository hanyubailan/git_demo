#ifndef DECODE_CAN_H
#define DECODE_CAN_H

#include "stm32f4xx_hal.h"
#define R  0.4f                   // �ֵ���뾶����λΪm
#define HF_track_length 3.0f      // ����뾶����λΪm   (��У��)

typedef struct {
    int16_t rotor_angle;        // ת�ӻ�е�Ƕ�
    int16_t RPM;                // ת��ת��
    int16_t torque_current;     // ʵ��ת�ص���
    uint8_t motor_temperature;  // ����¶�
	
    uint16_t last_ecd;        // ��һ�ζ�ȡ�ı�����ֵ
    uint16_t ecd;             // 0-8191,�̶��ܹ���8192��
    int32_t total_round;      // ��Ȧ��,ע�ⷽ��       
    float total_angle;        // �ܽǶ�,ע�ⷽ��	
    float total_distance;     // Ŀǰ�߹�����·��
	} CanData;

extern CanData decoded_data;   //��������ĵ����Ϣ
extern CanData motor1; 
extern CanData motor2; 
extern CanData motor3; 
extern CanData motor4; 
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
CanData decode_can_data(uint8_t data[8]);

#endif
