/*
 * speed.c
 *
 *  Created on: 2022Äê7ÔÂ13ÈÕ
 *      Author: ROY1994
 */
#include "speed.h"

extern PID PID_L, PID_R;
extern ImgInfoTypedef imgInfo;
extern ConstDataTypeDef ConstData;
extern SystemDataTypedef SystemData;

void speed_control(void)
{
    static int stoptarget = -60;
    if (SystemData.isStop == 'T')
    {
        if(stoptarget < 0) stoptarget+=10;
        PID_L.targetPoint = stoptarget;
        PID_R.targetPoint = stoptarget;
        PID_L.theoryTarget = stoptarget;
        PID_R.theoryTarget = stoptarget;
    }
    else if (imgInfo.straight_needSpeedUP == 'T')
    {
        PID_L.targetPoint = ConstData.speed.kMaxSpeed;
        PID_R.targetPoint = ConstData.speed.kMaxSpeed;
        PID_L.theoryTarget = ConstData.speed.kMaxSpeed;
        PID_R.theoryTarget = ConstData.speed.kMaxSpeed;
    }
    else if ((imgInfo.RoadType == Circle_L || imgInfo.RoadType == Circle_R) && imgInfo.CircleStatus != CIRCLE_OFF)
    {
        PID_L.targetPoint = ConstData.speed.kCircleSpeed;
        PID_R.targetPoint = ConstData.speed.kCircleSpeed;
        PID_L.theoryTarget = ConstData.speed.kCircleSpeed;
        PID_R.theoryTarget = ConstData.speed.kCircleSpeed;
    }
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
        PID_L.targetPoint = ConstData.speed.kBarnInSpeed;
        PID_R.targetPoint = ConstData.speed.kBarnInSpeed;
        PID_L.theoryTarget = ConstData.speed.kBarnInSpeed;
        PID_R.theoryTarget = ConstData.speed.kBarnInSpeed;
    }
    else if (imgInfo.RoadType == Barn_Out)
    {
        PID_L.targetPoint = ConstData.speed.kBarnOutSpeed;
        PID_R.targetPoint = ConstData.speed.kBarnOutSpeed;
        PID_L.theoryTarget = ConstData.speed.kBarnOutSpeed;
        PID_R.theoryTarget = ConstData.speed.kBarnOutSpeed;
    }
    else if (imgInfo.RoadType == Slope)
    {
        PID_L.targetPoint = ConstData.speed.kSlopeSpeed;
        PID_R.targetPoint = ConstData.speed.kSlopeSpeed;
        PID_L.theoryTarget = ConstData.speed.kSlopeSpeed;
        PID_R.theoryTarget = ConstData.speed.kSlopeSpeed;
    }
    else if (imgInfo.error >= 15)
    {
        PID_L.targetPoint = ConstData.speed.kTurnSpeed;
        PID_R.targetPoint = ConstData.speed.kTurnSpeed;
        PID_L.theoryTarget = ConstData.speed.kTurnSpeed;
        PID_R.theoryTarget = ConstData.speed.kTurnSpeed;
    }
    else
    {
        PID_L.targetPoint = ConstData.speed.kNormalSpeed;
        PID_R.targetPoint = ConstData.speed.kNormalSpeed;
        PID_L.theoryTarget = ConstData.speed.kNormalSpeed;
        PID_R.theoryTarget = ConstData.speed.kNormalSpeed;
    }
}


