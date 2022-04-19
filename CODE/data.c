/*
 * data.c
 *
 *  Created on: 2022Äê4ÔÂ9ÈÕ
 *      Author: ROY1994
 */
#include "data.h"
#include "pid.h"
#include "ImageDeal.h"
#include "ImagePreDeal.h"
#include "motor.h"
#include "steer.h"

ConstDataTypeDef ConstData;


void data_set(void)
{
    ConstData.kServoLowLimit = 1350;
    ConstData.kServoHighLimit = 1630;
    ConstData.kServoMid = 1490;

}
