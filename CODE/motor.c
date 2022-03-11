#include "motor.h"
#include "stdlib.h"
#include "math.h"

extern PID PID_L, PID_R;

void motor_init(void)
{
	gtm_pwm_init(MOTOR_RA, 3000, 0);
	gtm_pwm_init(MOTOR_RB, 3000, 0);
	gtm_pwm_init(MOTOR_LA, 3000, 0);
	gtm_pwm_init(MOTOR_LB, 3000, 0);

	pwm_duty(MOTOR_RA, 0);
	pwm_duty(MOTOR_RB, 0);
	pwm_duty(MOTOR_LA, 0);
	pwm_duty(MOTOR_LB, 0);

	gpt12_init(GPT12_T2, GPT12_T2INB_P33_7, GPT12_T2EUDB_P33_6);//左
	gpt12_init(GPT12_T4, GPT12_T4INA_P02_8, GPT12_T4EUDA_P00_9);//右
}
void servo_init(void)
{
    gtm_pwm_init(SERVO_PIN, 50, SERVO_MID);
}

void motor_control(void)
{
    int encoderL, encoderR, encoderLFilter, encoderRFilter, pwmL, pwmR;
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
//    pwm_duty(MOTOR_LA, 5000+pwmL);
//    pwm_duty(MOTOR_LB, 5000-pwmL);
//    pwm_duty(MOTOR_RA, 5000+pwmR);
//    pwm_duty(MOTOR_RB, 5000-pwmR);


    //vofa发送
#ifdef DEBUG_MOTOR_PID
    vofa_sendFloat((float)encoderL);
    vofa_sendFloat((float)encoderR);
    vofa_sendFloat((float)encoderLFilter);
    vofa_sendFloat((float)encoderRFilter);
    vofa_sendFloat((float)PID_L.targetPoint);
    vofa_sendFloat((float)PID_R.targetPoint);
    vofa_sendFloat(speedR);
    vofa_sendTail();
#endif

}
void servo_control(void)
{

}


