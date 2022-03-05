#include "pid.h"
#include "math.h"
#include "string.h"
/*
    .targetPoint = 300,
    .P = 8.5,
    .I = 0.83,
    .D = 5.0,
    .alphaDev = 0.7,
    .alphaOut = 0.7,

    .feedForwardK = 7.3236,
    .feedForwardB = 342.28,

 */
PID PID_L = {
    .targetPoint = 200,
    .P = 8.5,
    .I = 0.83,
    .D = 5.0,
    .alphaDev = 0.05,//0.3
    .alphaOut = 1.0,//0.2

    .feedForwardK = 7.3236,
    .feedForwardB = 342.28,

    .para = 0,
    .lastError = 0,
    .prevError = 0,
    .integralError = 0,

    .lastResult = 0,
    .result = 0,
};
PID PID_R = {
    .targetPoint = 200,
        .P = 9.0,
        .I = 0.9,
        .D = 5.0,
        .alphaDev = 0.05,
        .alphaOut = 1.0,

        .feedForwardK = 6.9979,
        .feedForwardB = 361.75,

        .para = 0,
        .lastError = 0,
        .prevError = 0,
        .integralError = 0,

        .lastResult = 0,
        .result = 0,
};

PID PID_Servo = {
    .targetPoint = 200,
    .P = 8.5,
    .I = 0.83,
    .D = 5.0,
    .alphaDev = 0.05,//0.3
    .alphaOut = 1.0,//0.2

    .feedForwardK = 7.3236,
    .feedForwardB = 342.28,

    .para = 0,
    .lastError = 0,
    .prevError = 0,
    .integralError = 0,

    .lastResult = 0,
    .result = 0,
};
void PID_initParam(PID *pid)
{
    memset(pid, 0, sizeof(pid));
}

inline void set_pid_targetPoint(PID *pid, int newTargetPoint)
{
    pid->targetPoint = newTargetPoint; // 设置当前的目标值
}

/**
  * @brief  获取目标值
  * @param  无
  * @note   无
  * @retval 目标值
  */
inline int get_pid_targetPoint(PID *pid)
{
    return pid->targetPoint; // 设置当前的目标值
}

/**
  * @brief  设置比例、积分、微分系数
  * @param  p：比例系数 P
  * @param  i：积分系数 i
  * @param  d：微分系数 d
  * @note   无
  * @retval 无
  */
inline void set_pid(PID *pid, float p, float i, float d)
{
    pid->P = p; // 设置比例系数 P
    pid->I = i; // 设置积分系数 I
    pid->D = d; // 设置微分系数 D
}

//增量式pid
int PID_calcInc(PID *pid, int nowPoint)
{
    int dError, feedForward;

    pid->iError = pid->targetPoint - nowPoint;

    if(pid->result > 4000) //判断上次结果是否最大/小值,若是,则对当前误差归0
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

//位置式pid
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
    //按照权重相加，可以去掉突变点。
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
    //按照权重相加，可以去掉突变点。
//    sum = EncoderDataR[0] * 0.999 + EncoderDataR[1] * 0.001;
    sum = EncoderDataR[0] * 0.05 + EncoderDataR[1] * 0.1 + EncoderDataR[2] * 0.15 + EncoderDataR[3] * 0.3 + EncoderDataR[4] * 0.4;

    return sum;
}


