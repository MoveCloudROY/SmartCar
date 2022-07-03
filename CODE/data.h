/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:50
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-02 04:50:46
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
    int kServoLowLimit, kServoHighLimit, kServoMid;
    float kImageStraightLineVarianceTh, kImageStraightLineSpeedUpVarianceTh;
    int kImageCircleOutVarianceTh;
    int kArcman;
    float kImageCircleInRepairLineK, kImageCircleOutRepairLineK;
    float kImageCrossIOUth;
}ConstDataTypeDef;


void data_set(void);

#endif /* CODE_DATA_H_ */
