#ifndef DECODE_CAN_H
#define DECODE_CAN_H

#include "stm32f4xx_hal.h"
#define R  0.4f                   // 轮电机半径，单位为m
#define HF_track_length 3.0f      // 滑轨半径，单位为m   (待校正)

typedef struct {
    int16_t rotor_angle;        // 转子机械角度
    int16_t RPM;                // 转子转速
    int16_t torque_current;     // 实际转矩电流
    uint8_t motor_temperature;  // 电机温度
	
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    int32_t total_round;      // 总圈数,注意方向       
    float total_angle;        // 总角度,注意方向	
    float total_distance;     // 目前走过的总路程
	} CanData;

extern CanData decoded_data;   //电调解码后的电机信息
extern CanData motor1; 
extern CanData motor2; 
extern CanData motor3; 
extern CanData motor4; 
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
CanData decode_can_data(uint8_t data[8]);

#endif
