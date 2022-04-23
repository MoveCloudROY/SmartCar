/*
 * @Author: ROY1994
 * @Date: 2022-02-04 14:01:20
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-04-14 21:22:00
 * @FilePath: \myImageDeal\ImageDeal.h
 * @Description: t
 */

#ifndef IMAGE_DEAL_H
#define IMAGE_DEAL_H

//#define __ON_PC__
// #define __ON_ROBOT__


#ifdef __ON_PC__
#include "ImagePreDeal.h"
#include "stack.h"
#include <cstdint>

#define uint8_t uint8
#define uint16_t uint16
#define int16_t int16
#define int8_t int8


#define DEBUG

#else
#include "headfile.h"
#include "ImagePreDeal.h"



#endif




#define MISS                                    255
#define LEFT_LIMIT                              2 //��4��Ϊ��߽�
#define RIGHT_LIMIT                             185 //��183��Ϊ��߽�
#define UP_LIMIT                                2
#define DOWN_LIMIT                              117
#define TH_ContinuityDelta                      10 //�����Բ�ֵ
#define TH_JumpOfDown                           4
#define TH_JumpOfUp                             8


// enum RoundAboutFlag{OFF, NOTICE, PASSONE, READYFORIN, TURNING, READYFOROUT, GO, };
// enum StartingLineFlag{OFF, FIRST, SECOND, THIRD, };
// enum ThreeForkRoadFlag{OFF, FIRST, SECOND, };

typedef enum _LineTypeEnum{LEFT, MID, RIGHT, }LineTypeEnum;
typedef enum _RoadTypeEnum{Straight, Cross, Slope, S_bend, Big_bend, Reflection, Round_About, Starting_Line, Fork_In, Fork_Out, Turn_Left, Turn_Right}RoadTypeEnum;
typedef enum _EdgePointTypeEnum {__OCCUPY,EXIST,LOST,JUMP} EdgePointTypeEnum;

// /**
//  * @brief ״̬��
//  * @description: ��Ϊ ����·�ڣ�б�£�s�䣬���䣬�����������ߣ�����·�� 7��Ԫ��   +   ���������б� 1������Ԫ��
//  */

/**
 * @brief ͼ�������Ϣ
 * 
 */
typedef struct _ImgInfoTypedef
{
    uint8_t top;
    uint8_t bottom;

    // float k;
    // float b;
    // uint8_t black_num;
    uint8_t leftDownJump, rightDownJump, leftUpJump, rightUpJump,leftDownStart, rightDownStart, leftUpStart, rightUpStart;
    uint8_t leftSeriesBreak, rightSeriesBreak;
    int error,allLostCnt;
    RoadTypeEnum RoadType;
}ImgInfoTypedef;

typedef struct _RowInfoTypedef
{
    uint8_t leftLine, rightLine, midLine, width;//ÿ���߶ȵ���/��/���߼����
    uint8_t fork_L, fork_R;
    int fork_blackWidth;
    float fork_black_k;
    EdgePointTypeEnum leftStatus, rightStatus;
    int error;
    LineTypeEnum LineType;
}RowInfoTypedef;

/**
 * @brief ���ص�����
 * @description: ͼ�������Ϸ�Ϊԭ�㣬x���������죬y����������
 */
typedef struct _PixelTypedef
{
    uint16_t x, y;
}PixelTypedef;

typedef struct _EdgePointTypedef
{
    int posX;
    EdgePointTypeEnum type;
}EdgePointTypedef;




//�����ú���
void img_process(void);

//��ʼ������
void params_init(void);

// ����ɨ��&��ȡ��������
void basic_searchLine(int bottom,int top);
void advance_searchLine(int bottom);
void advance_repairLine(void);
void advacnde_midLineFilter(void);
void basic_getSpecialParams(uint8_t select_begin, uint8_t select_end);
void basic_getJumpPointFromDet(uint8_t *row, int L,int R, EdgePointTypedef *Q, LineTypeEnum type);

void basic_repairLine(void);
void custom_repairLine(uint8_t select_top, uint8_t select_bottom, uint8_t apply_top, uint8_t apply_bottom, LineTypeEnum type);

// ������ɨ�ߺ���&��ȡ��������
void series_searchLine(void);
void series_getSpecialParams(void);

// ���е�·Ԫ��
void road_judge(void);


// �ر�Ԫ�ص�ɨ��
void turn_searchLine(void);
void cross_searchLine(void);
void cross_searchLine_withLeft(void);
void cross_searchLine_withRight(void);

void fork_searchLine(void);
void fork_repairLine(void);

void slope_searchLine(void);
void roundAbout_searchLine(void);
void startingLine_searchLine(void);



// ��ȡƫ��ֵ����PID
void get_error(void);

// �����Ժ���
float get_curvature(uint8_t select_top, uint8_t select_bottom, LineTypeEnum type);
uint8_t judge_lineContinuity(uint8_t select_begin, uint8_t select_end, LineTypeEnum type);
float calc_curvature(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3);
void add_line(float k, float b, uint8_t select_begin, uint8_t select_end, LineTypeEnum type);
void least_squares(float * k, float * b, uint8_t select_begin, uint8_t select_end, LineTypeEnum type);
inline float cosAOB(int xa, int ya, int xo, int yo, int xb, int yb);

// ���ҷ�����
// void find_RDJump(uint8_t start_point, uint8_t end_point);
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2);

#endif /*IMAGE_DEAL_H*/
