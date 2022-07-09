
/**********************************************************************************************************************
 * \file    ESKF.h
 * \brief
 * \version V1.0.0
 * \date    2022年6月24日
 * \author  McDuck
 *********************************************************************************************************************/


#ifndef CODE_ESKF_H_
#define CODE_ESKF_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "SEEKFREE_ICM20602.h"
/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define DATANUM                                         10                  //用于校准陀螺仪的数据采集次数
/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/
 // 系统标称状态
typedef struct{
    float quaternionI2G[4];
    float rotmatI2G[9];
    float eulerAngle[3];
    float positionAtG[3];
    float velocityAtG[3];
    float accelBias[3];
    float gyroBias[3];
    float gravityAtG[3];

    // 基于视觉信息计算的系统状态
//    float positionOnlyVisual[3];
//    float quaternionOnlyVisual[4];
}NominalState;
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

void ImuProcessing(void);

void InitializePose(void);

void ResetSystemState(void);

void UpdateNominalState(double dtime, float* accel, float* gyro);

void UpdateCovariance(double dtime, float* accel, float* gyro);

void ObservationUpdate(void);


#endif /* CODE_ESKF_H_ */
