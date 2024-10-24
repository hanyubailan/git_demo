#ifndef PID_H
#define PID_H

#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdlib.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID ������
    float Kp;
    float Ki;
    float Kd;

    int32_t max_out;  //������
    int32_t max_iout; //���������

    int32_t set;      //Ŀ��ֵ
    int32_t fdb;      //����ֵ

    int32_t out;
    int32_t Pout;
    int32_t Iout;
    int32_t Dout;
    int32_t Dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
    int32_t error[3]; //����� 0���� 1��һ�� 2���ϴ�

} pid_type_def;

/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      mode: PID_POSITION:��ͨPID
  *                 PID_DELTA: ���PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid������
  * @param[in]      max_iout: pid���������
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, float PID[3], int32_t max_out, int32_t max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid����
  * @param[out]     pid: PID�ṹ����ָ��
  * @param[in]      ref: ��������
  * @param[in]      set: �趨ֵ
  * @retval         pid���
  */
extern int32_t PID_calc(pid_type_def *pid, int32_t ref, int32_t set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ������
  * @param[out]     pid: PID�ṹ����ָ��
  * @retval         none
  */
	
extern void PID_clear(pid_type_def *pid);
extern int32_t MODE1_ChangeRPM(int32_t set, float ChangeRPM);
extern int32_t chang_sign(int32_t HubMotor);
#endif
