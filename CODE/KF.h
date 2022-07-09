
/**********************************************************************************************************************
 * \file    KF.h
 * \brief
 * \version V1.0.0
 * \date    2022��6��24��
 * \author  McDuck
 *********************************************************************************************************************/


#ifndef CODE_KF_H_
#define CODE_KF_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "headfile.h"
/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*-------------------------------------------------Data Structures---------------------------------------------------*/
/*********************************************************************************************************************/
 typedef struct{
     float eMea;    // Measure error
     float Kk;      // Kalman Gain
     float eEst;    // prior error
     float cEst;    // current estimate
     float lEst;    // last estimate
 }kalman_filter;

 typedef struct{
     float* p;          //position
     float* v;          //velocity
     float* q;          //quaternion
     float* R;          //rotation matrix
     float* a;
     float* w;
     float* g;
 }attitude;

/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/
//void icm_calibration(void);
float KF(float mData, kalman_filter* kf);
void calPos_RK4(void);

#endif /* CODE_KF_H_ */
