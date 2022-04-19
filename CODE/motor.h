#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "headfile.h"
#include "math.h"

#define MOTOR_RB        ATOM0_CH4_P02_4   //����2�����תPWM����
#define MOTOR_RA        ATOM0_CH5_P02_5   //����2�����תPWM����

#define MOTOR_LB        ATOM0_CH6_P02_6   //����2�����תPWM����
#define MOTOR_LA        ATOM0_CH7_P02_7   //����2�����תPWM����


#define PI                          3.14
#define CALC_SPD(x)                 ((float)(x)*PI/100.3) //(65mm * 10^-3) * PI * x/(1024 * 4) 5ms -> ��λm/s

void motor_init(void);
void motor_control(void);

#endif
