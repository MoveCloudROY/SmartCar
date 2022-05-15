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
}ConstDataTypeDef;


void data_set(void);

#endif /* CODE_DATA_H_ */
