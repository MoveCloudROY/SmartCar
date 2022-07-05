/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:58
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-04 22:10:19
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


void data_set(void)
{
    ConstData.kServoLowLimit = 1360;                        // 舵机右打角限制
    ConstData.kServoHighLimit = 1650;                       // 舵机中值(大致)
    ConstData.kServoMid = 1510  ;//1496;                    // 舵机左打角限制

    ConstData.kImageStraightLineVarianceTh = 26;            // 直线检测阈值
    ConstData.kImageStraightLineSpeedUpVarianceTh = 20;     // 直线加速检测阈值
    ConstData.kImageCircleOutVarianceTh = 200;              // 环岛出环线性方差阈值

    ConstData.kImageCircleInRepairLineK = 1.02;             // 环岛入环补线斜率
    ConstData.kImageCircleOutRepairLineK = 1.02;            // 环岛出环补线斜率

    ConstData.kImageCrossIOUth = 0.8;                       // 十字检测左右空白行交并比阈值
    ConstData.kImageStraightCurvTh = 0.001;                 // 直线曲率阈值

    ConstData.kImagePOutVarianceTh = 200;                   // 环岛出环线性方差阈值

    ConstData.kArcman = 10;

}
