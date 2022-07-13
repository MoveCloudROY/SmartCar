/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:58
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-12 17:58:27
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

    ConstData.kServoLowLimit = 1360;                        // ����Ҵ������
    ConstData.kServoHighLimit = 1650;                       // �����ֵ(����)
    ConstData.kServoMid = 1510  ;//1496;                    // �����������

    ConstData.kImageStraightLineVarianceTh = 26;            // ֱ�߼����ֵ
    ConstData.kImageStraightLineSpeedUpVarianceTh = 20;     // ֱ�߼��ټ����ֵ
    ConstData.kImageLineVarianceTh = 200;              // �����������Է�����ֵ

    ConstData.kImageCircleInRepairLineK = 1.1;             // �����뻷����б��
    ConstData.kImageCircleOutRepairLineK = 1.02;            // ������������б��

    ConstData.kImageBarnOutRepairLineK = 1.2;

    ConstData.kImageCrossIOUth = 0.8;                       // ʮ�ּ�����ҿհ��н�������ֵ
    ConstData.kImageStraightCurvTh = 0.001;                 // ֱ��������ֵ

    ConstData.kImagePOutVarianceTh = 200;                   // �����������Է�����ֵ
    ConstData.kImagePassingOffset = 10;

    ConstData.kImageForkInPicCnt = 25;
    ConstData.kImageForkInOutPicCnt = 40;

    ConstData.kArcman = 20;

    // ================= ϵͳ��ʼ������ ================//

    SystemData.isBarnOut = 'F';
    SystemData.isStop = 'F';

}

void reset_debugData(void)
{
    memset(&DebugData, 'F', sizeof(DebugData));
}
