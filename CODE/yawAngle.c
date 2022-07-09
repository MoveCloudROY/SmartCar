
/**********************************************************************************************************************
 * \file    yawAngle.c
 * \brief
 * \version V1.0.0
 * \date    2022��6��29��
 * \author  McDuck
 *********************************************************************************************************************/



/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "yawAngle.h"
/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
boolean angle_int_flag = 0;
float angle = 0;
extern float gyro[3];
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
/**
* @brief    ��ʼ���ֽǶ�
* @return   void
*/
void start_integrating_angle(void){
    angle_int_flag = 1;
}
/**
* @brief    ŷ�������ֽǶȣ������ж���
* @param    dt  ����ʱ����
* @return   void
*/
void interat_yaw_angle(float dt){
    if(angle_int_flag)
    {
        get_icm20602_gyro();
        angle += gyro[2]*dt;
    }
}
/**
* @brief    ��ѯ���ֽǶ�
* @return   ��ǰ���ֽǶ�
*/
inline float check_yaw_angle(){
    return angle;
}
/**
* @brief    ֹͣ���ֽǶȣ�����սǶ�ֵ
* @return   void
*/
void stop_interating_angle(void){
    angle_int_flag = 0;
    angle = 0;
}