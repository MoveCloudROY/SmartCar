/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:58
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-15 17:44:38
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

    // ================= 常数设置 ================ //
    ConstData.kServoMid = 1476  ;//1496;                    // 舵机左打角限制
    ConstData.kServoLowLimit = ConstData.kServoMid - SERVO_INTV;                        // 舵机右打角限制
    ConstData.kServoHighLimit = ConstData.kServoMid + SERVO_INTV;                       // 舵机中值(大致)


    // ================= 图像预处理常数设置 ================ //

    ConstData.kImageOtsuStaticTh = 95;
    ConstData.kImageOtsuBrightLimit = 200;

    // ================= 元素处理常数设置 ================ //

    ConstData.kImageStraightLineVarianceTh = 26;            // 直线检测阈值
    ConstData.kImageStraightLineSpeedUpVarianceTh = 20;     // 直线加速检测阈值
    ConstData.kImageLineVarianceTh = 200;              // 环岛出环线性方差阈值
    ConstData.kImageLineMeanVarianceTh = 30;

    ConstData.kImageCircleInRepairLineK = 1.02;             // 环岛入环补线斜率
    ConstData.kImageCircleOutRepairLineK = 1.2;            // 环岛出环补线斜率
    ConstData.kImageCircleInIntegralDis = 0.5;             // 环岛发现到入环路程积分
    ConstData.kImageCircleOffIntegralDis = 0.5;            // 环岛OFF到清空路程积分

    ConstData.kImageBarnOutRepairLineK = 0.9;
    ConstData.kImageBarnInRepairLineK = 3.0;

    ConstData.kImageCrossIOUth = 0.8;                       // 十字检测左右空白行交并比阈值
    ConstData.kImageStraightCurvTh = 0.001;                 // 直线曲率阈值

    ConstData.kImagePOutRepairLineK = 0.8;                 // P环出环补线斜率
    ConstData.kImagePOutVarianceTh = 200;                   // P环出环线性方差阈值
    ConstData.kImagePOutIntegralDis = 0.4;
    ConstData.kImagePassingOffset = 10;

//    ConstData.kImageForkInPicCnt = 25;
    ConstData.kImageForkIntegralDis = 1.0;

    ConstData.kImageBarnInFirIntegralDis = 0.8;
    ConstData.kImageBarnInSecIntegralDis = 0.8;

    ConstData.kSlopeUpAngRate = 2.0;
    ConstData.kSlopeDownAngRate = -2.0;
    ConstData.kSlopeIntegralDis = 0.15;

    //    ConstData.kArcman = 20;

    // ================= 速度初始化设置 ================= //

    ConstData.speed.kMaxSpeed = 200;
    ConstData.speed.kNormalSpeed = 135;
    ConstData.speed.kCircleSpeed = 135;
    ConstData.speed.kPSpeed = 135;
    ConstData.speed.kForkSpeed = 130;
    ConstData.speed.kBarnSpeed = 100;
    ConstData.speed.kTurnSpeed = 130;

//        ConstData.speed.kMaxSpeed =     100;
//        ConstData.speed.kNormalSpeed =  80;
//        ConstData.speed.kCircleSpeed =  80;
//        ConstData.speed.kPSpeed =       80;
//        ConstData.speed.kForkSpeed =    80;
//        ConstData.speed.kBarnSpeed =    80;
//        ConstData.speed.kTurnSpeed =    80;

    ConstData.speed.kSlopeSpeed = 80;
    ConstData.speed.kDiffAnglePerPWM = 0.217;


    ConstData.kBarnOutDegree = DEGREE_75;
    ConstData.kBarnInDegree = DEGREE_80;
    ConstData.kCircleInDegree = DEGREE_45;
    ConstData.kCircleOutDegree = DEGREE_320;
    ConstData.kPOutDegree = DEGREE_67;
    ConstData.kPPassingDegree = DEGREE_250;


    // ================= 系统初始化设置 ================= //

    SystemData.isBarnOut = 'F';
    SystemData.isStop = 'F';
    SystemData.isBuzzerOn = 'F';
    SystemData.barnInDetectCnt = 0;

}

void reset_debugData(void)
{
    memset(&DebugData, 'F', sizeof(DebugData));
}
