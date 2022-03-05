/*
 * @Author: ROY1994
 * @Date: 2022-02-04 14:01:20
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-03-05 11:40:38
 * @FilePath: \myImageDeal_v0.1\ImageDeal.h
 * @Description: t
 */

#ifndef IMAGE_DEAL_H
#define IMAGE_DEAL_H

#include "headfile.h"
#include "ImagePreDeal.h"
#include "stack.h"


#define MISS 									255
#define LEFT_LIMIT		 						2 //��4��Ϊ��߽�
#define RIGHT_LIMIT 							185 //��183��Ϊ��߽�
#define UP_LIMIT 								2
#define DOWN_LIMIT 								117
#define TH_ContinuityDelta 						10 //�����Բ�ֵ
#define TH_JumpOfDown							4
#define TH_JumpOfUp								8


// enum RoundAboutFlag{OFF, NOTICE, PASSONE, READYFORIN, TURNING, READYFOROUT, GO, };
// enum StartingLineFlag{OFF, FIRST, SECOND, THIRD, };
// enum ThreeForkRoadFlag{OFF, FIRST, SECOND, };

typedef enum _LineTypeEnum{LEFT, MID, RIGHT, } LineTypeEnum;

// enum RoadTypeEnum{Straight, Cross, Slope, S_bend, Big_bend, Reflection, Round_About, Starting_Line, Three_Fork_Road, Turn_Left, Turn_Right};


// /**
//  * @brief ״̬��
//  * @description: ��Ϊ ����·�ڣ�б�£�s�䣬���䣬�����������ߣ�����·�� 7��Ԫ��   +   ���������б� 1������Ԫ��
//  */

/**
 * @brief ͼ�������Ϣ
 * 
 */
typedef struct _imgInfo
{
	uint8 top;
    uint8 bottom;

	// float k;
	// float b;
	// uint8 black_num;
	uint8 len;
	int error;
	RoadTypeEnum RoadType;
}imgInfo;

/**
 * @brief ���ص�����
 * @description: ͼ�������Ϸ�Ϊԭ�㣬x���������죬y����������
 */
typedef struct _pixel
{
    uint16 x, y;
}pixel;


//�����ú���
void img_process(void);

//��ʼ������
void params_init(void);

// ����ɨ��&��ȡ��������
void basic_searchLine(void);
void basic_getSpecialParams(uint8 select_begin, uint8 select_end);
void basic_repairLine(void);
void advance_repairLine(uint8 select_top, uint8 select_bottom, uint8 apply_top, uint8 apply_bottom, LineTypeEnum type);


//���е�·Ԫ��
void road_judge(void);

//�ر�Ԫ�ص�ɨ��
void turn_searchLine(void);
void cross_searchLine(void);
void cross_searchLine_withLeft(void);
void cross_searchLine_withRight(void);

void slope_searchLine(void);
void roundAbout_searchLine(void);

//��ȡƫ��ֵ����PID
void get_error(void);

//�����Ժ���
float get_curvature(uint8 select_top, uint8 select_bottom, LineTypeEnum type);
uint8 judge_lineContinuiuint8(uint8 select_begin, uint8 select_end, LineTypeEnum type);
float calc_curvature(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3);
void add_line(float k, float b, uint8 select_begin, uint8 select_end, LineTypeEnum type);
void least_squares(float * k, float * b, uint8 select_begin, uint8 select_end, LineTypeEnum type);

//���ҷ�����
// void find_RDJump(uint8 start_point, uint8 end_point);
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2);

#endif /*IMAGE_DEAL_H*/
