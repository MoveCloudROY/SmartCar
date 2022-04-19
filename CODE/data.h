/*
 * data.h
 *
 *  Created on: 2022Äê4ÔÂ9ÈÕ
 *      Author: ROY1994
 */

#ifndef CODE_DATA_H_
#define CODE_DATA_H_
typedef struct _ConstDataTypeDef
{
        int kServoLowLimit, kServoHighLimit, kServoMid;
}ConstDataTypeDef;


void data_set(void);

#endif /* CODE_DATA_H_ */
