/*
 * system.h
 *
 *  Created on: 2022Äê4ÔÂ9ÈÕ
 *      Author: ROY1994
 */

#ifndef CODE_SYSTEM_H_
#define CODE_SYSTEM_H_

typedef struct _SystemStatusTypedef
{
    int cpu0_usage, cpu1_usage;

    int seconds;
}SystemStatusTypedef;


void car_init(void);
void car_backstage(void);
void img_backstage(void);
void car_statusbar(void);
#endif /* CODE_SYSTEM_H_ */
