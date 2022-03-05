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
#define LEFT_LIMIT		 						2 //以4作为左边界
#define RIGHT_LIMIT 							185 //以183作为左边界
#define UP_LIMIT 								2
#define DOWN_LIMIT 								117
#define TH_ContinuityDelta 						10 //连续性差值
#define TH_JumpOfDown							4
#define TH_JumpOfUp								8


// enum RoundAboutFlag{OFF, NOTICE, PASSONE, READYFORIN, TURNING, READYFOROUT, GO, };
// enum StartingLineFlag{OFF, FIRST, SECOND, THIRD, };
// enum ThreeForkRoadFlag{OFF, FIRST, SECOND, };

typedef enum _LineTypeEnum{LEFT, MID, RIGHT, } LineTypeEnum;

// enum RoadTypeEnum{Straight, Cross, Slope, S_bend, Big_bend, Reflection, Round_About, Starting_Line, Three_Fork_Road, Turn_Left, Turn_Right};


// /**
//  * @brief 状态机
//  * @description: 分为 交叉路口，斜坡，s弯，大弯，环岛，起跑线，三岔路口 7种元素   +   光照条件判别 1种特殊元素
//  */

/**
 * @brief 图像基本信息
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
 * @brief 像素点坐标
 * @description: 图像以左上方为原点，x轴向右延伸，y轴向下延伸
 */
typedef struct _pixel
{
    uint16 x, y;
}pixel;


//主调用函数
void img_process(void);

//初始化函数
void params_init(void);

// 常规扫线&获取参数函数
void basic_searchLine(void);
void basic_getSpecialParams(uint8 select_begin, uint8 select_end);
void basic_repairLine(void);
void advance_repairLine(uint8 select_top, uint8 select_bottom, uint8 apply_top, uint8 apply_bottom, LineTypeEnum type);


//特判道路元素
void road_judge(void);

//特别元素的扫线
void turn_searchLine(void);
void cross_searchLine(void);
void cross_searchLine_withLeft(void);
void cross_searchLine_withRight(void);

void slope_searchLine(void);
void roundAbout_searchLine(void);

//获取偏差值传至PID
void get_error(void);

//功能性函数
float get_curvature(uint8 select_top, uint8 select_bottom, LineTypeEnum type);
uint8 judge_lineContinuiuint8(uint8 select_begin, uint8 select_end, LineTypeEnum type);
float calc_curvature(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3);
void add_line(float k, float b, uint8 select_begin, uint8 select_end, LineTypeEnum type);
void least_squares(float * k, float * b, uint8 select_begin, uint8 select_end, LineTypeEnum type);

//暂且废弃的
// void find_RDJump(uint8 start_point, uint8 end_point);
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2);

#endif /*IMAGE_DEAL_H*/
