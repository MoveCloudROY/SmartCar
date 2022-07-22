
/**********************************************************************************************************************
 * \file    yawAngle.h
 * \brief
 * \version V1.0.0
 * \date    2022��6��29��
 * \author  McDuck
 *********************************************************************************************************************/


#ifndef CODE_YAWANGLE_H_
#define CODE_YAWANGLE_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "headfile.h"
/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define DEGREE_30   (PI/6.0f)
#define DEGREE_40   (PI/4.5f)
#define DEGREE_45   (PI/4.0f)
#define DEGREE_60   (PI/3.0f)
#define DEGREE_67   (PI/2.68f)
#define DEGREE_70   (PI/2.57f)
#define DEGREE_75   (PI/2.4f)
#define DEGREE_80   (4.0f * PI/9.0f)
#define DEGREE_90   (PI/2.0f)
#define DEGREE_160
#define DEGREE_180  (PI)
#define DEBGEE_230  (1.28f * PI)
#define DEGREE_250  (1.39f * PI)
#define DEGREE_270  (1.5f * PI)
#define DEGREE_360  (2.0f * PI)


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
extern volatile boolean angle_int_flag;
extern volatile float angle;
/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/
 
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
void start_integrating_angle(void);
void interat_yaw_angle(float dt);
float check_yaw_angle(void);
void stop_interating_angle(void);
uint8 is_interating_angle(void);
float check_pitch_rad(void);

#endif /* CODE_YAWANGLE_H_ */
