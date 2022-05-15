#include "motor.h"
#include "pid.h"
#include "stdlib.h"
#include "math.h"
#include "vofa.h"
#define DEBUG_MOTOR_PID
//#define OLTEST

PID PID_L = {
    .targetPoint = 200,
//    .P = 8.5,
//    .I = 0.83,
    .P = 29.5264604345801,
    .I = 2.22777631957277,
    .D = 8.51117071359754,
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
<<<<<<< HEAD
    .targetPoint = 50,
    .P = 9.0,
    .I = 0.9,
    .D = 5.0,
    .alphaDev = 0.05,
    .alphaOut = 0.1,
=======
        .targetPoint = 200,
//        .P = 9.0,
//        .I = 0.9,
        .P = 29.9297150662304,
        .I = 1.74447815226406,
        .D = 0,
        .alphaDev = 0.05,
        .alphaOut = 0.1,
>>>>>>> 2a00ab0d51f07133afcc3f18f5b81b02598bb338

    .feedForwardK = 6.9979,
    .feedForwardB = 361.75,

    .para = 0,
    .lastError = 0,
    .prevError = 0,
    .integralError = 0,

    .lastResult = 0,
    .result = 0,
};

<<<<<<< HEAD
float speedL, speedR;
=======
boolean intFlag = FALSE;
int disR = 0, disL = 0;
>>>>>>> 2a00ab0d51f07133afcc3f18f5b81b02598bb338

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

void startInt(void){
    intFlag = TRUE;
    disL = 0.0;
    disR = 0.0;
}

void stopInt(void){
    intFlag = FALSE;
}

void motor_control(void)
{
    int encoderL, encoderR, encoderLFilter, encoderRFilter, pwmL, pwmR;
<<<<<<< HEAD

=======
    float speedL, speedR,intL,intR;
>>>>>>> 2a00ab0d51f07133afcc3f18f5b81b02598bb338
    //int oldParaL, oldParaR, nowParaL, nowParaR;
    //float filterParam = 0.1;
#ifdef OLTEST
    pwm_duty(MOTOR_LA, 9000);
    pwm_duty(MOTOR_LB, 0);
    pwm_duty(MOTOR_RA, 9000);
    pwm_duty(MOTOR_RB, 0);
    encoderL = gpt12_get(GPT12_T2);
    encoderR = -gpt12_get(GPT12_T4);//右轮正转负的,所以手动取个反
    gpt12_clear(GPT12_T2);
    gpt12_clear(GPT12_T4);
    static long long Lsum=0;
    static long long Rsum=0;
    static double T=0;
//    Lsum+=encoderL;Rsum+=encoderR;
    T+=0.005;
    char tmpstr[100];
    sprintf(tmpstr,"%lf,%d,%d\n",T,encoderL, encoderR);
    uart_putstr(UART_0, tmpstr);
    return;
#endif
    //读取编码器
    encoderL = gpt12_get(GPT12_T2);
    encoderR = -gpt12_get(GPT12_T4);//右轮正转负的,所以手动取个反
    //对编码器读取值滤波
    gpt12_clear(GPT12_T2);
    gpt12_clear(GPT12_T4);

//    general_sendFloat((float)encoderL);
//    general_sendFloat((float)encoderR);
//    vofa_sendTail();
//  encoderLFilter = encoderL;
//  encoderRFilter = encoderR;
    encoderLFilter = recurrence_filter_left(encoderL);
    encoderRFilter = recurrence_filter_right(encoderR);

    speedL = CALC_SPD((abs(encoderLFilter)));
    speedR = CALC_SPD((abs(encoderRFilter)));
    //偏差计算
    pwmL = PID_calcInc(&PID_L, encoderLFilter);
    pwmR = PID_calcInc(&PID_R, encoderRFilter);
    //限幅
    if(pwmL > 4000)
        pwmL = 4000;
    else if(pwmL < -4000)
        pwmL = -4000;
    if(pwmL > 4000)
        pwmL = 4000;
    else if(pwmL < -4000)
        pwmL = -4000;

    if(pwmR > 4000)
        pwmR = 4000;
    else if(pwmR < -4000)
        pwmR = -4000;
    if(pwmR > 4000)
        pwmR = 4000;
    else if(pwmR < -4000)
        pwmR = -4000;
//    nowParaL = (float)filterParam * oldParaL + (1.0 - filterParam) * paraL;
//    nowParaR = (float)filterParam * oldParaR + (1.0 - filterParam) * paraR;
//    oldParaL = nowParaL;
//    oldParaR = nowParaR;

    //设定PWM值
    pwm_duty(MOTOR_LA, 5000+pwmL);
    pwm_duty(MOTOR_LB, 5000-pwmL);
    pwm_duty(MOTOR_RA, 5000+pwmR);
    pwm_duty(MOTOR_RB, 5000-pwmR);


    if(intFlag){
        disL += speedL*0.005;
        disR += speedR*0.005;
    }

    //vofa发送
#ifdef DEBUG_MOTOR_PID
    general_sendFloat((float)encoderL);
    general_sendFloat((float)encoderR);
    general_sendFloat((float)encoderLFilter);
    general_sendFloat((float)encoderRFilter);
    general_sendFloat((float)PID_L.targetPoint);
    general_sendFloat((float)PID_R.targetPoint);
    general_sendFloat((float)PID_L.result);
    general_sendFloat((float)PID_R.result);
    general_sendFloat(speedL);
    general_sendFloat(speedR);
    vofa_sendTail();
#endif

}


