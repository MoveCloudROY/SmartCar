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

    // ================= �������� ================ //
    ConstData.kServoMid = 1476  ;//1496;                    // �����������
    ConstData.kServoLowLimit = ConstData.kServoMid - SERVO_INTV;                        // ����Ҵ������
    ConstData.kServoHighLimit = ConstData.kServoMid + SERVO_INTV;                       // �����ֵ(����)


    // ================= ͼ��Ԥ���������� ================ //

    ConstData.kImageOtsuStaticTh = 95;
    ConstData.kImageOtsuBrightLimit = 200;

    // ================= Ԫ�ش��������� ================ //

    ConstData.kImageStraightLineVarianceTh = 26;            // ֱ�߼����ֵ
    ConstData.kImageStraightLineSpeedUpVarianceTh = 20;     // ֱ�߼��ټ����ֵ
    ConstData.kImageLineVarianceTh = 200;              // �����������Է�����ֵ
    ConstData.kImageLineMeanVarianceTh = 30;

    ConstData.kImageCircleInRepairLineK = 1.02;             // �����뻷����б��
    ConstData.kImageCircleOutRepairLineK = 1.2;            // ������������б��
    ConstData.kImageCircleInIntegralDis = 0.5;             // �������ֵ��뻷·�̻���
    ConstData.kImageCircleOffIntegralDis = 0.5;            // ����OFF�����·�̻���

    ConstData.kImageBarnOutRepairLineK = 0.9;
    ConstData.kImageBarnInRepairLineK = 3.0;

    ConstData.kImageCrossIOUth = 0.8;                       // ʮ�ּ�����ҿհ��н�������ֵ
    ConstData.kImageStraightCurvTh = 0.001;                 // ֱ��������ֵ

    ConstData.kImagePOutRepairLineK = 0.8;                 // P����������б��
    ConstData.kImagePOutVarianceTh = 200;                   // P���������Է�����ֵ
    ConstData.kImagePOutIntegralDis = 0.4;
    ConstData.kImagePassingOffset = 10;

//    ConstData.kImageForkInPicCnt = 25;
    ConstData.kImageForkIntegralDis = 1.0;

    ConstData.kImageBarnInFirIntegralDis = 0.8;
    ConstData.kImageBarnInSecIntegralDis = 0.8;

    ConstData.kSlopeUpAngRate = 2.0;
    ConstData.kSlopeDownAngRate = -2.0;
    ConstData.kSlopeIntegralDis = 0.15;

    //    ConstData.kArcman = 20;

    // ================= �ٶȳ�ʼ������ ================= //

    ConstData.speed.kMaxSpeed = 200;
    ConstData.speed.kNormalSpeed = 135;
    ConstData.speed.kCircleSpeed = 135;
    ConstData.speed.kPSpeed = 135;
    ConstData.speed.kForkSpeed = 130;
    ConstData.speed.kBarnSpeed = 100;
    ConstData.speed.kTurnSpeed = 130;

//        ConstData.speed.kMaxSpeed =     100;
//        ConstData.speed.kNormalSpeed =  80;
//        ConstData.speed.kCircleSpeed =  80;
//        ConstData.speed.kPSpeed =       80;
//        ConstData.speed.kForkSpeed =    80;
//        ConstData.speed.kBarnSpeed =    80;
//        ConstData.speed.kTurnSpeed =    80;

    ConstData.speed.kSlopeSpeed = 80;
    ConstData.speed.kDiffAnglePerPWM = 0.217;


    ConstData.kBarnOutDegree = DEGREE_75;
    ConstData.kBarnInDegree = DEGREE_80;
    ConstData.kCircleInDegree = DEGREE_45;
    ConstData.kCircleOutDegree = DEGREE_320;
    ConstData.kPOutDegree = DEGREE_67;
    ConstData.kPPassingDegree = DEGREE_250;


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
