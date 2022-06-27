/*
 * steer.c
 *
 *  Created on: 2022年4月9日
 *      Author: ROY1994
 */

#include "headfile.h"
#include "data.h"
#include "ImageDeal.h"
#include "steer.h"
#include "pid.h"

extern ImgInfoTypedef imgInfo;
extern ConstDataTypeDef ConstData;

#define STEER_LIMIT_LOW(pwm) ((pwm)<ConstData.kServoLowLimit?(ConstData.kServoLowLimit):(pwm))
#define STEER_LIMIT_HIGH(pwm) ((pwm)>ConstData.kServoHighLimit?(ConstData.kServoHighLimit):(pwm))

PID PID_Servo = {
    .targetPoint = 0,
    .P = 4,
    .I = 0,
    .D = 1,
    .alphaDev = 0.0,
    .alphaOut = 0.0,

    .feedForwardK = 0.0,

    .para = 0,
    .lastError = 0,
    .prevError = 0,
    .integralError = 0,

    .lastResult = 0,
    .result = 0,
};

int steer_pwm;

void servo_init(void)
{
    gtm_pwm_init(SERVO_PIN, 50, SERVO_MID);
}
void servo_set(int duty)
{
    pwm_duty(SERVO_PIN, duty);
}
void servo_control_PIDPos(void)
{
    PID_Servo.iError = imgInfo.error;

    int Outpid =    PID_Servo.P * PID_Servo.iError
                    +  PID_Servo.D * (PID_Servo.iError - PID_Servo.lastError);

    PID_Servo.lastError = PID_Servo.iError;

    steer_pwm = ConstData.kServoMid - Outpid;

    steer_pwm = STEER_LIMIT_LOW(steer_pwm);
    steer_pwm = STEER_LIMIT_HIGH(steer_pwm);
    servo_set(steer_pwm);
//    servo_set(ConstData.kServoMid);
//    vofa_sendFloat((float)steer_pwm);
//    vofa_sendTail();
}

///****************************     阿克曼加减差速      ****************************/
//void CS_control(int setv)
//{
//    int a = 0;
//    float Temp_Orr = 0;
//
//    if(steer_pwm > ConstData.kServoMid)   //右转
//    {
//        a = (int16)(42*(ConstData.kArcman)*1.0/(ConstData.kServoHighLimit - ConstData.kServoMid)) ;   //angle_max   40/
//        if(a > 42)  a = 42;
//        if(a < 0)   a = 0;
//        Temp_Orr = tan((a*3.14)/180) * 13.4 / 40;          //15.4
//        PID_L.targetPoint = setv * (1.0 + PID_Servo.P * Temp_Orr); //第一个常数可以加大差速，第二个常数可以提前差速 //原来是0.78
//        PID_R.targetPoint = setv * (1.0 - PID_Servo.P * Temp_Orr);//原来是1
//    }
//    else
//    {
//        a=(int16)(42*(-ConstData.kArcman)*1.0/(ConstData.kServoMid - ConstData.kServoLowLimit)) ;  //angle_max   40
//        if(a > 42)  a=42;
//        if(a < 0)   a=0;
//        Temp_Orr = tan((a*3.14)/180) * 13.4 / 40;
//        PID_L.targetPoint = setv * (1.0 - PID_Servo.P * Temp_Orr); //原来是1
//        PID_R.targetPoint = setv * (1.0 + PID_Servo.P * Temp_Orr);//原来是0.78
//    }
//
//    if(setv==0)
//    {
//        PID_L.targetPoint = 0;
//        PID_R.targetPoint = 0;
//    }
//}
