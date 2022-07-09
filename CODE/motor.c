#include "motor.h"
#include "pid.h"
#include "stdlib.h"
#include "math.h"
#include "vofa.h"


PID PID_L = {
    .targetPoint = 200,
    .P = 24.5045390601805,
    .I = 0.005*216.937391941517,
    .D = 0,
    .alphaDev = 0.05,//0.3
    .alphaOut = 0.1,//0.2

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
    .P = 20.3031449809298,
    .I = 0.005*268.373269126206,
    .D = 0,
    .alphaDev = 0.05,
    .alphaOut = 0.1,

    .feedForwardK = 6.9979,
    .feedForwardB = 361.75,

    .para = 0,
    .lastError = 0,
    .prevError = 0,
    .integralError = 0,

    .lastResult = 0,
    .result = 0,
};

void motor_init(void)
{
	gtm_pwm_init(MOTOR_RA, 17000, 0);
	gtm_pwm_init(MOTOR_RB, 17000, 0);
	gtm_pwm_init(MOTOR_LA, 17000, 0);
	gtm_pwm_init(MOTOR_LB, 17000, 0);

	pwm_duty(MOTOR_RA, 0);
	pwm_duty(MOTOR_RB, 0);
	pwm_duty(MOTOR_LA, 0);
	pwm_duty(MOTOR_LB, 0);

	gpt12_init(GPT12_T2, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6);//左
	gpt12_init(GPT12_T4, GPT12_T4INA_P02_8, GPT12_T4EUDA_P00_9);//右
}


void motor_control(void)
{
    int encoderL, encoderR, encoderLFilter, encoderRFilter;
    static int pwmL = 0, pwmR = 0;
    float speedL, speedR;
    //int oldParaL, oldParaR, nowParaL, nowParaR;
    //float filterParam = 0.1;

    //读取编码器
    encoderL = gpt12_get(GPT12_T2);
    encoderR = -gpt12_get(GPT12_T4);//右轮正转负的,所以手动取个反
    //对编码器读取值滤波
    gpt12_clear(GPT12_T2);
    gpt12_clear(GPT12_T4);

//  encoderLFilter = encoderL;
//  encoderRFilter = encoderR;
    encoderLFilter = recurrence_filter_left(encoderL);
    encoderRFilter = recurrence_filter_right(encoderR);

    speedL = CALC_SPD(abs(encoderLFilter));
    speedR = CALC_SPD(abs(encoderRFilter));
    //偏差计算
    pwmL = PID_calcInc(&PID_L, encoderLFilter);
    pwmR = PID_calcInc(&PID_R, encoderRFilter);
    //限幅
    if(pwmL > 9500)
        pwmL = 9500;
    else if(pwmL < -9500)
        pwmL = -9500;
    if(pwmL > 9500)
        pwmL = 9500;
    else if(pwmL < -9500)
        pwmL = -9500;

    if(pwmR > 9500)
        pwmR = 9500;
    else if(pwmR < -9500)
        pwmR = -9500;
    if(pwmR > 9500)
        pwmR = 9500;
    else if(pwmR < -9500)
        pwmR = -9500;
//    nowParaL = (float)filterParam * oldParaL + (1.0 - filterParam) * paraL;
//    nowParaR = (float)filterParam * oldParaR + (1.0 - filterParam) * paraR;
//    oldParaL = nowParaL;
//    oldParaR = nowParaR;

    //设定PWM值
    if(pwmL >= 0){
        pwm_duty(MOTOR_LA, pwmL);
        pwm_duty(MOTOR_LB, 0);
    }else {
        pwm_duty(MOTOR_LA, GTM_ATOM0_PWM_DUTY_MAX + pwmL);
        pwm_duty(MOTOR_LB, GTM_ATOM0_PWM_DUTY_MAX);
    }
    if(pwmR >= 0){
        pwm_duty(MOTOR_RA, pwmR);
        pwm_duty(MOTOR_RB, 0);
    }else {
        pwm_duty(MOTOR_RA, GTM_ATOM0_PWM_DUTY_MAX + pwmR);
        pwm_duty(MOTOR_RB, GTM_ATOM0_PWM_DUTY_MAX);
    }



    //vofa发送
#ifdef DEBUG_MOTOR_PID
//    general_sendFloat((float)encoderL);
//    general_sendFloat((float)encoderR);
    general_sendFloat((float)encoderLFilter);
    general_sendFloat((float)encoderRFilter);
    general_sendFloat((float)pwmL);
    general_sendFloat((float)pwmR);
    general_sendFloat((float)PID_L.targetPoint);
    general_sendFloat((float)PID_R.targetPoint);
    general_sendFloat(speedR);
    vofa_sendTail();
#endif

}


