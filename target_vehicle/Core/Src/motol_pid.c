#include "motol_pid.h"

void  LimitMax(int32_t *input,int32_t max)   
    {                          
        if (*input > max)       
        {                      
            *input = max;       
        }                      
        else if (*input < - max) 
        {                      
            *input = - max;     
        }                      
    };

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
	
void PID_init(pid_type_def *pid, uint8_t mode, float PID[3], int32_t max_out, int32_t max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

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
int32_t PID_calc(pid_type_def *pid, int32_t ref, int32_t set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)                            // ��������Ĺ�ʽpid_out = Kp * E + Ki * (E�ۺ�) + Kd * (E1 - E0)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];      
        LimitMax(&(pid->Iout), pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;

    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);   //���ڣ�E1-E2��-��E2-E3��
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(&(pid->out), pid->max_out);
    }
    return (int16_t)pid->out;
}

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
void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

/**
  * @brief          ģʽ1�µľ���ֵ�ļӼ�����͹��㴦��
  */
int32_t MODE1_ChangeRPM(int32_t set, float ChangeRPM)
{
    int sign = (set < 0) ? -1 : 1;  // ��ȡԭʼ���ķ���
    int32_t abs_set = abs(set);       // ��ȡԭʼ���ľ���ֵ
    
    // �����仯���ӻ���������ҶԽ����������
    int32_t modified_abs_value = abs_set + ChangeRPM;

    // ����ӷ������ֵ���� 3000������Ϊ 3000
    if (modified_abs_value > 5000) {
        modified_abs_value = 5000;
    }

    // ����ӷ������ֵС�� 0������Ϊ 0
    if (modified_abs_value < 0) {
        modified_abs_value = 0;
    }
		
    return modified_abs_value * sign;
};

int32_t chang_sign(int32_t HubMotor)    // ��ת���
{
	return HubMotor*(-1);
};
