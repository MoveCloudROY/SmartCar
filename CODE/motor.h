#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "headfile.h"
#include "math.h"

#define MOTOR_RB        ATOM0_CH4_P02_4   //定义2电机正转PWM引脚
#define MOTOR_RA        ATOM0_CH5_P02_5   //定义2电机反转PWM引脚

#define MOTOR_LB        ATOM0_CH6_P02_6   //定义2电机正转PWM引脚
#define MOTOR_LA        ATOM0_CH7_P02_7   //定义2电机反转PWM引脚


#define PI                          3.14
#define CALC_SPD(x)                 ((float)(x)*PI/100.3) //(65mm * 10^-3) * PI * x/(1024 * 4) 5ms -> 单位m/s

void motor_init(void);
void motor_control(void);

#endif
