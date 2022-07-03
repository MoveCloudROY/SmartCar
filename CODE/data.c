/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:58
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-02 05:19:12
 * @FilePath: \myImageDeal\data.cpp
 * @Description:
 */
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

    ConstData.kImageCircleInRepairLineK = 1.02;
    ConstData.kImageCircleOutRepairLineK = 1.02;

    ConstData.kImageCrossIOUth = 0.8;
    ConstData.kArcman = 10;

}
