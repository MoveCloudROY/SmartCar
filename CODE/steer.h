/*
 * steer.h
 *
 *  Created on: 2022年4月9日
 *      Author: ROY1994
 */

#ifndef CODE_STEER_H_
#define CODE_STEER_H_

#include "pid.h"
#include "motor.h"

#define SERVO_PIN       ATOM1_CH1_P33_9       //定义舵机引脚
#define SERVO_MID       1500

extern PID PID_L;
extern PID PID_R;

void servo_init(void);
void servo_set(int duty);
void servo_control_PIDPos(void);
void differential_speed(int pwm_diff);

#endif /* CODE_STEER_H_ */
