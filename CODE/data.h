/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:50
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-13 15:25:08
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

#if defined (__ON_ROBOT__)

#include "pid.h"
#include "ImageDeal.h"
#include "ImagePreDeal.h"
#include "motor.h"
#include "steer.h"

#else
#include <string.h>
#endif

#define SERVO_INTV 150

typedef struct _SpeedSubDataTypeDef
{
    int     kMaxSpeed;
    int     kNormalSpeed;
    int     kCircleSpeed;
    int     kPSpeed;
    int     kForkSpeed;
    float   kDiffAnglePerPWM;
}SpeedSubDataTypeDef;

typedef struct _ConstDataTypeDef
{
    int     kServoLowLimit, kServoHighLimit, kServoMid;
    float   kImageStraightLineVarianceTh, kImageStraightLineSpeedUpVarianceTh;
    int     kImageLineVarianceTh;
    float   kImageCircleInRepairLineK, kImageCircleOutRepairLineK;
    float   kImageBarnOutRepairLineK;
    float   kImageCrossIOUth;
    float   kImageStraightCurvTh;
    float   kImagePOutRepairLineK;
    int     kImagePOutVarianceTh;
    int     kImagePassingOffset;
    int     kImageForkInPicCnt, kImageForkInOutPicCnt;
//    int     kArcman;
    SpeedSubDataTypeDef speed;

}ConstDataTypeDef;

typedef struct _SystemDataTypedef
{
    char    isBarnOut;
    char    isStop;
    char    isBuzzerOn;
}SystemDataTypedef;

typedef struct _DebugDataTypedef
{
        char PFlagInRange;
        char PFlagVariOK;
        float SteerR;
        float SteerAngle;
}DebugDataTypedef;

void data_set(void);
void reset_debugData(void);

#endif /* CODE_DATA_H_ */
