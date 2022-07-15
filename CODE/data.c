/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:58
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-15 17:44:38
 * @FilePath: \myImageDeal\data.cpp
 * @Description:
 */
/*
 * data.c
 *
 *  Created on: 2022��4��9��
 *      Author: ROY1994
 */
#include "data.h"


ConstDataTypeDef ConstData;
SystemDataTypedef SystemData;
DebugDataTypedef DebugData;

void data_set(void)
{

    // ================= �������� ================//
    ConstData.kServoMid = 1490  ;//1496;                    // �����������
    ConstData.kServoLowLimit = ConstData.kServoMid - SERVO_INTV;                        // ����Ҵ������
    ConstData.kServoHighLimit = ConstData.kServoMid + SERVO_INTV;                       // �����ֵ(����)


    ConstData.kImageStraightLineVarianceTh = 26;            // ֱ�߼����ֵ
    ConstData.kImageStraightLineSpeedUpVarianceTh = 20;     // ֱ�߼��ټ����ֵ
    ConstData.kImageLineVarianceTh = 200;              // �����������Է�����ֵ

    ConstData.kImageCircleInRepairLineK = 1.1;             // �����뻷����б��
    ConstData.kImageCircleOutRepairLineK = 1.02;            // ������������б��

    ConstData.kImageBarnOutRepairLineK = 1.2;

    ConstData.kImageCrossIOUth = 0.8;                       // ʮ�ּ�����ҿհ��н�������ֵ
    ConstData.kImageStraightCurvTh = 0.001;                 // ֱ��������ֵ

    ConstData.kImagePOutRepairLineK =0.7;                 // P����������б��
    ConstData.kImagePOutVarianceTh = 200;                   // P���������Է�����ֵ
    ConstData.kImagePassingOffset = 10;

    ConstData.kImageForkInPicCnt = 25;
    ConstData.kImageForkInOutPicCnt = 40;

    ConstData.kImageBarnInPicCnt = 20;

//    ConstData.kArcman = 20;

    // ================= �ٶȳ�ʼ������ ================= //

    ConstData.speed.kMaxSpeed = 140;
    ConstData.speed.kNormalSpeed = 120;
    ConstData.speed.kCircleSpeed = 110;
    ConstData.speed.kPSpeed = 110;
    ConstData.speed.kForkSpeed = 100;
    ConstData.speed.kDiffAnglePerPWM = 0.2;


    // ================= ϵͳ��ʼ������ ================= //

    SystemData.isBarnOut = 'F';
    SystemData.isStop = 'F';
    SystemData.isBuzzerOn = 'F';
    SystemData.barnInDetectCnt = 0;

}

void reset_debugData(void)
{
    memset(&DebugData, 'F', sizeof(DebugData));
}
