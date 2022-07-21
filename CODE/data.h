/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:50
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-15 17:43:44
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

#include <stdint.h>
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
    int     kBarnInSpeed;
    int     kTurnSpeed;
    float   kDiffAnglePerPWM;
}SpeedSubDataTypeDef;

typedef struct _ConstDataTypeDef
{
    int     kServoLowLimit, kServoHighLimit, kServoMid;

    int     kImageOtsuStaticTh, kImageOtsuBrightLimit;

    float   kImageStraightLineVarianceTh, kImageStraightLineSpeedUpVarianceTh;
    int     kImageLineVarianceTh;
    float   kImageCircleInRepairLineK, kImageCircleOutRepairLineK;
    float   kImageCircleInIntegralDis, kImageCircleOffIntegralDis;
    float   kImageBarnOutRepairLineK;
    float   kImageBarnInRepairLineK;
    float   kImageCrossIOUth;
    float   kImageStraightCurvTh;
    float   kImagePOutRepairLineK;
    float   kImagePOutIntegralDis;
    int     kImagePOutVarianceTh;
    int     kImagePassingOffset;
    float   kImageForkIntegralDis;
    float   kImageBarnInFirIntegralDis, kImageBarnInSecIntegralDis;
//    int     kArcman;
    SpeedSubDataTypeDef speed;

}ConstDataTypeDef;

typedef struct _SystemDataTypedef
{
    char    isBarnOut;
    char    isBarnIn;
    char    isStop;
    char    isBuzzerOn;
    uint8_t barnInDetectCnt;
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
