/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:50
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-04 22:07:20
 * @FilePath: \myImageDeal\data.h
 * @Description:
 */
/*
 * data.h
 *
 *  Created on: 2022Äê4ÔÂ9ÈÕ
 *      Author: ROY1994
 */

#ifndef CODE_DATA_H_
#define CODE_DATA_H_

#if _ON_PC_

#include "pid.h"
#include "ImageDeal.h"
#include "ImagePreDeal.h"
#include "motor.h"
#include "steer.h"

#else

#endif


typedef struct _ConstDataTypeDef
{
    int     kServoLowLimit, kServoHighLimit, kServoMid;
    float   kImageStraightLineVarianceTh, kImageStraightLineSpeedUpVarianceTh;
    int     kImageCircleOutVarianceTh;
    float   kImageCircleInRepairLineK, kImageCircleOutRepairLineK;
    float   kImageCrossIOUth;
    float   kImageStraightCurvTh;
    int     kImagePOutVarianceTh;
    int     kArcman;
}ConstDataTypeDef;


void data_set(void);

#endif /* CODE_DATA_H_ */
