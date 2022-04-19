/*
 * steer.h
 *
 *  Created on: 2022年4月9日
 *      Author: ROY1994
 */

#ifndef CODE_STEER_H_
#define CODE_STEER_H_


#define SERVO_PIN       ATOM1_CH1_P33_9       //定义舵机引脚
#define SERVO_MID       1500


void servo_init(void);
void servo_set(int duty);
void servo_control_PIDPos(void);

#endif /* CODE_STEER_H_ */
