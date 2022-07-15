/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:58
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-13 15:24:37
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
    ConstData.kServoMid = 1510  ;//1496;                    // �����������
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

//    ConstData.kArcman = 20;

    // ================= �ٶȳ�ʼ������ ================= //

    ConstData.speed.kMaxSpeed = 170;
    ConstData.speed.kNormalSpeed = 150;
    ConstData.speed.kCircleSpeed = 140;
    ConstData.speed.kPSpeed = 140;
    ConstData.speed.kForkSpeed = 130;
    ConstData.speed.kDiffAnglePerPWM = 0.17;


    // ================= ϵͳ��ʼ������ ================= //

    SystemData.isBarnOut = 'F';
    SystemData.isStop = 'F';
    SystemData.isBuzzerOn = 'F';

}

void reset_debugData(void)
{
    memset(&DebugData, 'F', sizeof(DebugData));
}
