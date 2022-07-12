/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:50
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-12 17:58:37
 * @FilePath: \myImageDeal\data.h
 * @Description:
 */
/*
 * data.h
 *
 *  Created on: 2022��4��9��
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
    int     kImageLineVarianceTh;
    float   kImageCircleInRepairLineK, kImageCircleOutRepairLineK;
    float   kImageBarnOutRepairLineK;
    float   kImageCrossIOUth;
    float   kImageStraightCurvTh;
    int     kImagePOutVarianceTh;
    int     kImagePassingOffset;
    int     kImageForkInPicCnt, kImageForkInOutPicCnt;
    int     kArcman;

}ConstDataTypeDef;

typedef struct _SystemDataTypedef
{
    char    isBarnOut;
    char    isStop;
}SystemDataTypedef;

void data_set(void);

#endif /* CODE_DATA_H_ */
