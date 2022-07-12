/*
 * @Author: ROY1994
 * @Date: 2022-04-23 21:14:58
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-11 21:15:47
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

}
