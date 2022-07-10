#ifndef _PID_H_
#define	_PID_H_
#include "headfile.h"
#include <stdio.h>
#include <stdlib.h>


typedef struct _PID
{
    int targetPoint;                    //目标值
    int theoryTarget;                   //理论目标值（未经过差速处理）
    int para;        		            //增量
    int iError;             			//偏差值
    int lastError;          		    //上一个偏差值
	int prevError;                      //上上一次偏差值
    float P, I, D;                      //比例、积分、微分系数
    float alphaDev;                        //微分滤波系数
    float alphaOut;
    float feedForwardK, feedForwardB;
    int lastDev;                        //上一次微分项值
    int integralError;                  //积分值

    int lastResult;
    int result;                         //输出值
}PID;

void PID_initParam(PID *pid);
void set_pid_targetPoint(PID *pid, int newTargetPoint);
int  get_pid_targetPoint(PID *pid);
void set_pid(PID *pid, float p, float i, float d);
int  PID_calcInc(PID *pid, int nowPoint);
int  PID_calcPos(PID *pid, int nowPoint);
int  recurrence_filter_left(int nowEncoder);
int  recurrence_filter_right(int nowEncoder);

#endif /*_PID_H_*/
