#ifndef _PID_H_
#define	_PID_H_
#include "headfile.h"
#include <stdio.h>
#include <stdlib.h>


typedef struct _PID
{
    int targetPoint;                    //Ŀ��ֵ
    int theoryTarget;                   //����Ŀ��ֵ��δ�������ٴ���
    int para;        		            //����
    int iError;             			//ƫ��ֵ
    int lastError;          		    //��һ��ƫ��ֵ
	int prevError;                      //����һ��ƫ��ֵ
    float P, I, D;                      //���������֡�΢��ϵ��
    float alphaDev;                        //΢���˲�ϵ��
    float alphaOut;
    float feedForwardK, feedForwardB;
    int lastDev;                        //��һ��΢����ֵ
    int integralError;                  //����ֵ

    int lastResult;
    int result;                         //���ֵ
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
