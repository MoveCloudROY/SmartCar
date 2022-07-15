/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:58
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-13 15:24:37
 * @FilePath: \myImageDeal\data.cpp
 * @Description:
 */
/*
 * data.c
 *
 *  Created on: 2022年4月9日
 *      Author: ROY1994
 */
#include "data.h"


ConstDataTypeDef ConstData;
SystemDataTypedef SystemData;
DebugDataTypedef DebugData;

void data_set(void)
{

    // ================= 常数设置 ================//
    ConstData.kServoMid = 1510  ;//1496;                    // 舵机左打角限制
    ConstData.kServoLowLimit = ConstData.kServoMid - SERVO_INTV;                        // 舵机右打角限制
    ConstData.kServoHighLimit = ConstData.kServoMid + SERVO_INTV;                       // 舵机中值(大致)


    ConstData.kImageStraightLineVarianceTh = 26;            // 直线检测阈值
    ConstData.kImageStraightLineSpeedUpVarianceTh = 20;     // 直线加速检测阈值
    ConstData.kImageLineVarianceTh = 200;              // 环岛出环线性方差阈值

    ConstData.kImageCircleInRepairLineK = 1.1;             // 环岛入环补线斜率
    ConstData.kImageCircleOutRepairLineK = 1.02;            // 环岛出环补线斜率

    ConstData.kImageBarnOutRepairLineK = 1.2;

    ConstData.kImageCrossIOUth = 0.8;                       // 十字检测左右空白行交并比阈值
    ConstData.kImageStraightCurvTh = 0.001;                 // 直线曲率阈值

    ConstData.kImagePOutRepairLineK =0.7;                 // P环出环补线斜率
    ConstData.kImagePOutVarianceTh = 200;                   // P环出环线性方差阈值
    ConstData.kImagePassingOffset = 10;

    ConstData.kImageForkInPicCnt = 25;
    ConstData.kImageForkInOutPicCnt = 40;

//    ConstData.kArcman = 20;

    // ================= 速度初始化设置 ================= //

    ConstData.speed.kMaxSpeed = 170;
    ConstData.speed.kNormalSpeed = 150;
    ConstData.speed.kCircleSpeed = 140;
    ConstData.speed.kPSpeed = 140;
    ConstData.speed.kForkSpeed = 130;
    ConstData.speed.kDiffAnglePerPWM = 0.17;


    // ================= 系统初始化设置 ================= //

    SystemData.isBarnOut = 'F';
    SystemData.isStop = 'F';
    SystemData.isBuzzerOn = 'F';

}

void reset_debugData(void)
{
    memset(&DebugData, 'F', sizeof(DebugData));
}
