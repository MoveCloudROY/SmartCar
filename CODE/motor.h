#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "headfile.h"
#include "pid.h"
#include "data.h"
#include "stdlib.h"
#include "math.h"

#define MOTOR_RB        ATOM0_CH4_P02_4   //����2�����תPWM����
#define MOTOR_RA        ATOM0_CH5_P02_5   //����2�����תPWM����

#define MOTOR_LB        ATOM0_CH6_P02_6   //����2�����תPWM����
#define MOTOR_LA        ATOM0_CH7_P02_7   //����2�����תPWM����


#define CALC_SPD(x)                 ((float)(x)*PI* 65.0 *(30.0/68)* 1e-3 * 200.0/(1024.0)) //(65mm * 10^-3) * PI * x*(30/68)*200/(1024 * 4) 5ms -> ��λm/s


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
