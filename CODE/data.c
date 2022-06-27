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
    ConstData.kServoLowLimit = 1360;
    ConstData.kServoHighLimit = 1650;
    ConstData.kServoMid = 1510  ;//1496;

    ConstData.kImageStraightLineVarianceTh = 26;
    ConstData.kImageStraightLineSpeedUpVarianceTh = 20;
    ConstData.kImageCircleOutVarianceTh = 200;

    ConstData.kArcman = 10;
}
