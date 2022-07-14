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

void speed_control(void)
{
    if (imgInfo.straight_needSpeedUP == 'T')
    {
        PID_L.targetPoint = ConstData.speed.kMaxSpeed;
        PID_R.targetPoint = ConstData.speed.kMaxSpeed;
    }
    else if (imgInfo.RoadType == Circle_L || imgInfo.RoadType == Circle_R)
    {
        PID_L.targetPoint = ConstData.speed.kCircleSpeed;
        PID_R.targetPoint = ConstData.speed.kCircleSpeed;
    }
    else if (imgInfo.RoadType == P_L || imgInfo.RoadType == P_R)
    {
        PID_L.targetPoint = ConstData.speed.kPSpeed;
        PID_R.targetPoint = ConstData.speed.kPSpeed;
    }
    else if (imgInfo.RoadType == Fork_In || imgInfo.RoadType == Fork_Out)
    {
        PID_L.targetPoint = ConstData.speed.kForkSpeed;
        PID_R.targetPoint = ConstData.speed.kForkSpeed;
    }
    else
    {
        PID_L.targetPoint = ConstData.speed.kNormalSpeed;
        PID_R.targetPoint = ConstData.speed.kNormalSpeed;
    }
}


