#include "pid.h"
#include "math.h"
#include "string.h"

void PID_initParam(PID *pid)
{
    memset(pid, 0, sizeof(pid));
}

inline void set_pid_targetPoint(PID *pid, int newTargetPoint)
{
    pid->targetPoint = newTargetPoint; // ���õ�ǰ��Ŀ��ֵ
}

/**
  * @brief  ��ȡĿ��ֵ
  * @param  ��
  * @note   ��
  * @retval Ŀ��ֵ
  */
inline int get_pid_targetPoint(PID *pid)
{
    return pid->targetPoint; // ���õ�ǰ��Ŀ��ֵ
}

/**
  * @brief  ���ñ��������֡�΢��ϵ��
  * @param  p������ϵ�� P
  * @param  i������ϵ�� i
  * @param  d��΢��ϵ�� d
  * @note   ��
  * @retval ��
  */
inline void set_pid(PID *pid, float p, float i, float d)
{
    pid->P = p; // ���ñ���ϵ�� P
    pid->I = i; // ���û���ϵ�� I
    pid->D = d; // ����΢��ϵ�� D
}

//����ʽpid
int PID_calcInc(PID *pid, int nowPoint)
{
    int dError, feedForward;

    pid->iError = pid->targetPoint - nowPoint;

    if(pid->result > 4000) //�ж��ϴν���Ƿ����/Сֵ,����,��Ե�ǰ����0
    {
        if(pid->iError > 0)
        {
            pid->iError = 0;
        }
    }
    else if(pid->result < -4000)
    {
        if(pid->iError < 0)
        {
            pid->iError = 0;
        }
    }
    dError = pid->iError - 2 * pid->lastError + pid->prevError;
    pid->lastDev = (pid->D) * pid->alphaDev * (float)dError + (1.0f - pid->alphaDev) * pid->lastDev;


    pid->para =     pid->P * (pid->iError - pid->lastError)
                    + pid->I * pid->iError
                    + pid->lastDev;

    pid->prevError = pid->lastError;
    pid->lastError = pid->iError;

    if(pid->para <= 10 && pid->para >= -10)
        pid->para = 0;
    pid->result += pid->para;


    pid->result = pid->alphaOut * (float)pid->result + (1.0f - pid->alphaOut) * pid->lastResult;
    pid->lastResult = pid->result;
    return pid->result;
}


//λ��ʽpid
int PID_calcPos(PID *pid, int nowPoint)
{
    pid->iError = pid->targetPoint - nowPoint;
    pid->integralError += pid->iError;

    if(pid->integralError > 3600)
        pid->integralError = 3600;
    else if(pid->integralError < -3600)
        pid->integralError = -3600;

    int Outpid =    pid->P * pid->iError
                    +  pid->I * pid->integralError
                    +  pid->D * (pid->iError - pid->lastError);

    pid->lastError = pid->iError;

    return Outpid;
}
int EncoderDataL[10], EncoderDataR[10];
#define FILTER_WINDOW 5
int recurrence_filter_left(int nowEncoder)
{
    static int first = 1;
    int i;
    int sum = 0;
    if(first)
    {
        first = 0;
        for (i = 1; i < FILTER_WINDOW; i++)
            EncoderDataL[i - 1] = nowEncoder;
    }
    else
    {
        for (i = 1; i < FILTER_WINDOW; i++)
            EncoderDataL[i - 1] = EncoderDataL[i];
    }
    EncoderDataL[FILTER_WINDOW - 1] = nowEncoder;
    sum = 0;
    //����Ȩ����ӣ�����ȥ��ͻ��㡣
//    sum = EncoderDataL[0] * 0.999 + EncoderDataL[1] * 0.001;
    sum = EncoderDataL[0] * 0.05 + EncoderDataL[1] * 0.1 + EncoderDataL[2] * 0.15 + EncoderDataL[3] * 0.3 + EncoderDataL[4] * 0.4;

    return sum;
}
int recurrence_filter_right(int nowEncoder)
{
    static int first = 1;
    int i;
    int sum = 0;
    if(first)
    {
        first = 0;
        for (i = 1; i < FILTER_WINDOW; i++)
            EncoderDataR[i - 1] = nowEncoder;
    }
    else
    {
        for (i = 1; i < FILTER_WINDOW; i++)
            EncoderDataR[i - 1] = EncoderDataR[i];
    }
    EncoderDataR[FILTER_WINDOW - 1] = nowEncoder;
    sum = 0;
    //����Ȩ����ӣ�����ȥ��ͻ��㡣
//    sum = EncoderDataR[0] * 0.999 + EncoderDataR[1] * 0.001;
    sum = EncoderDataR[0] * 0.05 + EncoderDataR[1] * 0.1 + EncoderDataR[2] * 0.15 + EncoderDataR[3] * 0.3 + EncoderDataR[4] * 0.4;

    return sum;
}


