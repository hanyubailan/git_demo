//#ifndef __MOTOR_PID_H
//#include "stm32f4xx_hal.h"
////pid结构体
//typedef struct
//{
//    int32_t e;          // 偏差量error
//    int32_t e_pre;      // 上次偏差量
//    int32_t now;        // 现实量
//    int32_t target;     // 目标量
//    int32_t p_out;      // 比例
//    int32_t i_out;      // 积分
//    int32_t d_out;      // 微分
//    float Kp;           // 比例系数 控制调节快慢
//    float Ki;           // 积分系数 控制最终误差
//    float Kd;           // 微分系数 控制过调摆动
//    int32_t Ilimit;     // 积分分离
//    int32_t Outlimit;   // 输出限制
//    int16_t totalout;   // 总输出
//} _pid;
////

////初始化pid参数
//void pid_init(_pid *pid,uint32_t Ilimit,float Kp,float Ki,float Kd,uint32_t outlimit );
//void pid_calq (_pid *pid);

//#endif
/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
	
	
	
	
#ifndef PID_H
#define PID_H
#include "stm32f4xx_hal.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    int32_t max_out;  //最大输出
    int32_t max_iout; //最大积分输出

    int32_t set;      //目标值
    int32_t fdb;      //返还值

    int32_t out;
    int32_t Pout;
    int32_t Iout;
    int32_t Dout;
    int32_t Dbuf[3];  //微分项 0最新 1上一次 2上上次
    int32_t error[3]; //误差项 0最新 1上一次 2上上次

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
  * @param[out]     pid: PID结构数据指针
  * @param[in]      mode: PID_POSITION:普通PID
  *                 PID_DELTA: 差分PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid最大输出
  * @param[in]      max_iout: pid最大积分输出
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
  * @brief          pid计算
  * @param[out]     pid: PID结构数据指针
  * @param[in]      ref: 反馈数据
  * @param[in]      set: 设定值
  * @retval         pid输出
  */
extern int32_t PID_calc(pid_type_def *pid, int32_t ref, int32_t set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid 输出清除
  * @param[out]     pid: PID结构数据指针
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

#endif
