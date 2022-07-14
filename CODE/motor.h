#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "headfile.h"
#include "pid.h"
#include "data.h"
#include "stdlib.h"
#include "math.h"

#define MOTOR_RB        ATOM0_CH4_P02_4   //定义2电机正转PWM引脚
#define MOTOR_RA        ATOM0_CH5_P02_5   //定义2电机反转PWM引脚

#define MOTOR_LB        ATOM0_CH6_P02_6   //定义2电机正转PWM引脚
#define MOTOR_LA        ATOM0_CH7_P02_7   //定义2电机反转PWM引脚


#define CALC_SPD(x)                 ((float)(x)*PI* 65.0 *(30.0/68)* 1e-3 * 200.0/(1024.0)) //(65mm * 10^-3) * PI * x*(30/68)*200/(1024 * 4) 5ms -> 单位m/s


typedef struct _PassDisTypedef
{
    boolean intFlag;
    float disL, disR;
    void (*start)(struct _PassDisTypedef *);
    void (*stop)(struct _PassDisTypedef *);

}PassDisTypedef;



void motor_init(void);
void startIntDis(PassDisTypedef *);
void stopIntDis(PassDisTypedef *);
void motor_control(void);
void motor_stop(void);

#endif
