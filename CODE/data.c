/*
 * data.c
 *
 *  Created on: 2022Äê4ÔÂ9ÈÕ
 *      Author: ROY1994
 */
#include "data.h"


ConstDataTypeDef ConstData;


void data_set(void)
{
    ConstData.kServoLowLimit = 1356;
    ConstData.kServoHighLimit = 1636;
    ConstData.kServoMid = 1496;//1490;

    ConstData.kImageStraightLineVarianceTh = 26;
    ConstData.kImageStraightLineSpeedUpVarianceTh = 20;
    ConstData.kImageCircleOutVarianceTh = 200;

    ConstData.kArcman = 10;
}
