/*
 * @Author: ROY1994
 * @Date: 2022-02-04 14:01:20
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-15 16:43:22
 * @FilePath: \myImageDeal\ImageDeal.h
 * @Description: t
 */

#ifndef IMAGE_DEAL_H
#define IMAGE_DEAL_H

#include <stdint.h>
//#define __ON_PC__
#define __ON_ROBOT__


#ifdef __ON_PC__
#include "ImagePreDeal.h"
#include <cstdint>

#define DEBUG
#define DEBUG_DEAD() do{printf("\n\n\n!!DEADDEADDEADDEADDEADDEADDEADDEADDEADDEADDEADDEADDEADDEADDEADDEAD!!\n\n\n");}while(0)
#else
#include "headfile.h"
#include "ImagePreDeal.h"
#include "yawAngle.h"
#include "data.h"
#include "pid.h"
#include "buzzer.h"

#define uint8_t uint8
#define uint16_t uint16
#define int16_t int16
#define int8_t int8

extern DebugDataTypedef DebugData;
extern PID PID_L, PID_R;

// ========== Control ========= //
#define __ROBOT_DEBUG__
#define __BARN_OUT_ON__
#define __BARN_LEFT_OUT__


// ============================ //
#define DEBUG_DEAD()
#endif

#include "data.h"


#define MISS                                    255
#define LEFT_LIMIT                              2 //锟斤拷4锟斤拷为锟斤拷呓锟�
#define RIGHT_LIMIT                             185 //锟斤拷183锟斤拷为锟斤拷呓锟�
#define UP_LIMIT                                2
#define DOWN_LIMIT                              117
#define TH_ContinuityDelta                      10 //锟斤拷锟斤拷锟皆诧拷值
#define TH_JumpOfDown                           4
#define TH_JumpOfUp                             8


//===================================================================================================================//

// enum RoundAboutFlag{OFF, NOTICE, PASSONE, READYFORIN, TURNING, READYFOROUT, GO, };
// enum StartingLineFlag{OFF, FIRST, SECOND, THIRD, };
// enum ThreeForkRoadFlag{OFF, FIRST, SECOND, };

typedef enum _LineTypeEnum{LEFT, MID, RIGHT, }LineTypeEnum;
typedef enum _RoadTypeEnum{Road_None, Straight, Cross, Slope, P_L, P_R, Reflection, Circle_L, Circle_R, Starting_Line, Fork_In, Fork_Out, Barn_In, Barn_Out}RoadTypeEnum;
typedef enum _EdgePointTypeEnum {__EdgePointTypeEnum_OCCUPY, EXIST, LOST, JUMP} EdgePointTypeEnum;
typedef enum _CircleStatusTypeEnum {CIRCLE_NOT_FIND, CIRCLE_FIND, CIRCLE_IN, CIRCLE_PASSING, CIRCLE_OUT, CIRCLE_OFF} CircleStatusTypeEnum;
typedef enum _PStatusTypeEnum {P_NOT_FIND, P_PASSING, P_OUT_READY, P_OUT_1, P_OUT_2, P_OFF} PStatusTypeEnum;
typedef enum _DirTypeEnum{BlackToWhite, WhiteToBlack} DirTypeEnum;

//===================================================================================================================//

// /**
//  * @brief 状态锟斤拷
//  * @description: 锟斤拷为 锟斤拷锟斤拷路锟节ｏ拷斜锟铰ｏ拷s锟戒，锟斤拷锟戒，锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟竭ｏ拷锟斤拷锟斤拷路锟斤拷 7锟斤拷元锟斤拷   +   锟斤拷锟斤拷锟斤拷锟斤拷锟叫憋拷 1锟斤拷锟斤拷锟斤拷元锟斤拷
//  */

/**
 * @brief 图锟斤拷锟斤拷锟斤拷锟较�
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
    uint8_t straight_needSpeedUP;
    RoadTypeEnum RoadType;
    CircleStatusTypeEnum CircleStatus;
    PStatusTypeEnum PStatus;

    /*****锟斤拷路锟斤拷锟斤拷锟斤拷锟斤拷锟较�*****/

}ImgInfoTypedef;

typedef struct _RowInfoTypedef
{
    uint8_t leftLine, rightLine, midLine, width;//每锟斤拷锟竭度碉拷锟斤拷/锟斤拷/锟斤拷锟竭硷拷锟斤拷锟斤拷
    uint8_t fork_L, fork_R;
    int fork_blackWidth;
    float fork_black_k;
    EdgePointTypeEnum leftStatus, rightStatus;
    int error;
    LineTypeEnum LineType;
}RowInfoTypedef;

/**
 * @brief 锟斤拷锟截碉拷锟斤拷锟斤拷
 * @description: 图锟斤拷锟斤拷锟斤拷锟较凤拷为原锟姐，x锟斤拷锟斤拷锟斤拷锟斤拷锟届，y锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
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

typedef struct _EdgeJumpPointTypedef
{
    int posY;
    DirTypeEnum type;
}EdgeJumpPointTypedef;

typedef struct _BlackBlockTopTypedef
{
    int posY;
    int posX;
}BlackBlockTopTypedef;


//===================================================================================================================//

#if defined (DEBUG)
typedef struct _DebugVaribleTypedef
{
    int blackBlock_Hrow, blackBlock_Hcol;
}DebugVaribleTypedef;
#endif


//===================================================================================================================//

//锟斤拷锟斤拷锟矫猴拷锟斤拷
void img_process(void);

//锟斤拷始锟斤拷锟斤拷锟斤拷
void params_init(void);

// 锟斤拷锟斤拷扫锟斤拷&锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷
void basic_searchLine(int bottom,int top);
void advance_searchLine(int bottom);
void advance_repairLine(void);
void advance_midLineFilter(void);
void basic_getSpecialParams(uint8_t select_begin, uint8_t select_end);
void basic_getJumpPointFromDet(uint8_t *row, int L,int R, EdgePointTypedef *Q, LineTypeEnum type);

void basic_repairLine(void);
void custom_repairLine(uint8_t select_top, uint8_t select_bottom, uint8_t apply_top, uint8_t apply_bottom, LineTypeEnum type);

// 锟斤拷锟斤拷锟斤拷扫锟竭猴拷锟斤拷&锟斤拷取锟斤拷锟斤拷锟斤拷锟斤拷
void series_searchLine(void);
void series_getSpecialParams(void);

// 锟斤拷锟叫碉拷路元锟斤拷
void road_judge(void);


// 锟截憋拷元锟截碉拷扫锟斤拷
void straight_detect(void);
void straight_speedUpDetect(void);
void turn_detect(void);
void cross_detect(void);

void fork_detect(void);
void fork_repairLine(void);

void circle_judge_1(void);
void circle_judge_2(void);

void circle_judge_in(void);
void circle_detect(void);
void circle_repairLine(void);

void slope_detect(void);
void startingLine_detect(void);

void p_detect(void);
void p_repairLine(void);

void barnOut_repairLine(void);

void barnIn_detect(void);
void barnIn_repairLine(void);

uint8_t stop_detect(void);

// 锟斤拷取偏锟斤拷值锟斤拷锟斤拷PID
void get_error(void);
void calc_globalError(void);

// 锟斤拷锟斤拷锟皆猴拷锟斤拷
int get_variance(uint8_t select_top, uint8_t select_bottom, LineTypeEnum type);
int get_circleTop(void);
uint8_t judge_lineBeginLost(LineTypeEnum type);
float get_curvature(uint8_t select_top, uint8_t select_bottom, LineTypeEnum type);
uint8_t judge_lineContinuity(uint8_t select_begin, uint8_t select_end, LineTypeEnum type);
float calc_curvature(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3);
void add_line(float k, float b, uint8_t select_begin, uint8_t select_end, LineTypeEnum type);
void least_squares(float * k, float * b, uint8_t select_begin, uint8_t select_end, LineTypeEnum type);
inline float cosAOB(int xa, int ya, int xo, int yo, int xb, int yb);
inline void recalc_line(uint8_t select_top, uint8_t select_bottom, LineTypeEnum type);
// 锟斤拷锟揭凤拷锟斤拷锟斤拷
// void find_RDJump(uint8_t start_point, uint8_t end_point);
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2);
void perspective_transform(int raw, int col, float* xpos, float* ypos);

#endif /*IMAGE_DEAL_H*/
