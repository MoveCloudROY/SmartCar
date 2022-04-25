/*
 * steer.c
 *
 *  Created on: 2022Äê4ÔÂ9ÈÕ
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
    .D = 4,
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

    int pwm = ConstData.kServoMid - Outpid;

    pwm = STEER_LIMIT_LOW(pwm);
    pwm = STEER_LIMIT_HIGH(pwm);
    servo_set(pwm);
//    vofa_sendFloat((float)pwm);
//    vofa_sendTail();
}
