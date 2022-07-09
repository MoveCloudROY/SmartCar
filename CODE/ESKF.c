
/**********************************************************************************************************************
 * \file    ESKF.c
 * \brief
 * \version V1.0.0
 * \date    2022年6月24日
 * \author  McDuck
 *********************************************************************************************************************/



/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "ESKF.h"

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
extern int16 icm_gyro_x,icm_gyro_y,icm_gyro_z;
extern int16 icm_acc_x,icm_acc_y,icm_acc_z;
NominalState sysNominalState_={.quaternionI2G[0]=1.0,.quaternionI2G[0]=0.0,.quaternionI2G[0]=0.0,.quaternionI2G[0]=0.0};
/*********************************************************************************************************************/
/*--------------------------------------------Private Variables/Constants--------------------------------------------*/
/*********************************************************************************************************************/

/*********************************************************************************************************************/
/*------------------------------------------------Function Prototypes------------------------------------------------*/
/*********************************************************************************************************************/

static inline float int162float(int16 num){
    return (float)num / (num >= 0 ? 32767 : 32768);
}

static inline float vecnorm(float* vec){
    return sqrt(vec[0]*vec[0]+vec[1]*vec[1]+vec[2]*vec[2]);
}

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/
void InitializePose(void){
    // 计算accelBias gyroBias gravityAtG
    int16 asumx=0, asumy=0, asumz=0, gsumx=0, gsumy=0, gsumz=0;
    for(int i = 0; i < DATANUM; i++){
        get_icm20602_accdata();
        get_icm20602_gyro();
        asumx += icm_acc_x;
        asumy += icm_acc_y;
        asumz += icm_acc_z;
        gsumx += icm_gyro_x;
        gsumy += icm_gyro_y;
        gsumz += icm_gyro_z;
    }
    sysNominalState_.gyroBias[0] = int162float(gsumx) / DATANUM;
    sysNominalState_.gyroBias[1] = int162float(gsumy) / DATANUM;
    sysNominalState_.gyroBias[2] = int162float(gsumz) / DATANUM;
    sysNominalState_.accelBias[0] = int162float(asumx) / DATANUM;
    sysNominalState_.accelBias[1] = int162float(asumy) / DATANUM;
    sysNominalState_.accelBias[2] = int162float(asumz) / DATANUM;

    sysNominalState_.gravityAtG[2] = -sqrt((int162float(asumx) / DATANUM)*(int162float(asumx) / DATANUM)+
                                           (int162float(asumy) / DATANUM)*(int162float(asumy) / DATANUM)+
                                           (int162float(asumz) / DATANUM)*(int162float(asumz) / DATANUM));

}


void ImuProcessing(void){
    //这里最好互斥量获取数据
    //获取数据
    get_icm20602_accdata();
    get_icm20602_gyro();
    float accel[3] = {int162float(icm_acc_x),int162float(icm_acc_y),int162float(icm_acc_z)};
    float gyro[3]  = {int162float(icm_gyro_x),int162float(icm_gyro_y),int162float(icm_gyro_z)};
    float dt = 0.005;
    UpdateCovariance(dt, accel, gyro);
}

void UpdateCovariance(double dtime, float* accel, float* gyro){


//    // IMU数据
//       Eigen::Vector3d angularVel = gyro - this->sysNominalState_.gyroBias;
//       Eigen::Vector3d accelrate = accel - this->sysNominalState_.accelBias;
//
//       // 构造误差状态Jocobian矩阵
//       this->sysErrorState_.stateJocobianMat = Eigen::Matrix<double, 18, 18>::Identity();
//       this->sysErrorState_.stateJocobianMat.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * deltaT;
//       this->sysErrorState_.stateJocobianMat.block<3, 3>(3, 6) = -this->sysNominalState_.rotmatI2G * FBUSEKF::SkewSymmetricMatrix(accelrate) * deltaT;
//       this->sysErrorState_.stateJocobianMat.block<3, 3>(3, 9) = -this->sysNominalState_.rotmatI2G * deltaT;
//       this->sysErrorState_.stateJocobianMat.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * deltaT;
//       this->sysErrorState_.stateJocobianMat.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - FBUSEKF::SkewSymmetricMatrix(angularVel) * deltaT;
//       this->sysErrorState_.stateJocobianMat.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * deltaT;
//
//       // 主协方差矩阵更新
//       this->sysErrorState_.stateCovariance.block<18, 18>(0, 0) = this->sysErrorState_.stateJocobianMat * this->sysErrorState_.stateCovariance.block<18, 18>(0, 0) *
//                                                                      this->sysErrorState_.stateJocobianMat.transpose() +
//                                                                  this->sysErrorState_.noiseCoefficientMat * this->sysErrorState_.noiseCovariance *
//                                                                      this->sysErrorState_.noiseCoefficientMat.transpose();
//
//
//       // 进一步保证协方差矩阵的对称性
//       Eigen::MatrixXd stateCovFixed = (this->sysErrorState_.stateCovariance + this->sysErrorState_.stateCovariance.transpose()) / 2.0;
//       this->sysErrorState_.stateCovariance = stateCovFixed;


}

void UpdateNominalState(double dtime, float* accel, float* gyro){

}
