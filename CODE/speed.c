/*
 * speed.c
 *
 *  Created on: 2022��7��13��
 *      Author: ROY1994
 */
#include "speed.h"

extern PID PID_L, PID_R;
extern ImgInfoTypedef imgInfo;
extern ConstDataTypeDef ConstData;
extern SystemDataTypedef SystemData;

void speed_control(void)
{
    if (imgInfo.straight_needSpeedUP == 'T')
    {
        PID_L.targetPoint = ConstData.speed.kMaxSpeed;
        PID_R.targetPoint = ConstData.speed.kMaxSpeed;
        PID_L.theoryTarget = ConstData.speed.kMaxSpeed;
        PID_R.theoryTarget = ConstData.speed.kMaxSpeed;
    }
//    else if ((imgInfo.RoadType == Circle_L || imgInfo.RoadType == Circle_R) && imgInfo.CircleStatus != CIRCLE_OFF)
//    {
//        PID_L.targetPoint = ConstData.speed.kCircleSpeed;
//        PID_R.targetPoint = ConstData.speed.kCircleSpeed;
//        PID_L.theoryTarget = ConstData.speed.kCircleSpeed;
//        PID_R.theoryTarget = ConstData.speed.kCircleSpeed;
//    }
    else if ((imgInfo.RoadType == P_L || imgInfo.RoadType == P_R) && imgInfo.PStatus != P_PASSING)
    {
        PID_L.targetPoint = ConstData.speed.kPSpeed;
        PID_R.targetPoint = ConstData.speed.kPSpeed;
        PID_L.theoryTarget = ConstData.speed.kPSpeed;
        PID_R.theoryTarget = ConstData.speed.kPSpeed;
    }
    else if (imgInfo.RoadType == Fork_In || imgInfo.RoadType == Fork_Out)
    {
        PID_L.targetPoint = ConstData.speed.kForkSpeed;
        PID_R.targetPoint = ConstData.speed.kForkSpeed;
        PID_L.theoryTarget = ConstData.speed.kForkSpeed;
        PID_R.theoryTarget = ConstData.speed.kForkSpeed;
    }
    else if (imgInfo.RoadType == Barn_In)
    {
        PID_L.targetPoint = 100;
        PID_R.targetPoint = 100;
        PID_L.theoryTarget = 100;
        PID_R.theoryTarget = 100;
    }
    else if (SystemData.isStop == 'T')
    {
        PID_L.targetPoint = 0;
        PID_R.targetPoint = 0;
        PID_L.theoryTarget = 0;
        PID_R.theoryTarget = 0;
    }
    else
    {
        PID_L.targetPoint = ConstData.speed.kNormalSpeed;
        PID_R.targetPoint = ConstData.speed.kNormalSpeed;
        PID_L.theoryTarget = ConstData.speed.kNormalSpeed;
        PID_R.theoryTarget = ConstData.speed.kNormalSpeed;
    }
}


