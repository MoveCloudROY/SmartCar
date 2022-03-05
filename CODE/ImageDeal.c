/*
 * @Author: ROY1994
 * @Date: 2022-02-04 14:01:30
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-03-05 11:40:18
 * @FilePath: \myImageDeal_v0.1\ImageDeal.cpp
 * @Description: 搜线，元素判断等主要处理函数
 */
#include "ImageDeal.h"

//#define DEBUG

extern uint8 mt9v03x_image[HEIGHT][WIDTH];//原灰度图
extern uint8 imageBin[HEIGHT][WIDTH];//二值化图像

pixel leftLineSerial[HEIGHT<<1],rightLineSerial[HEIGHT<<1];//种子生长法所需参数
int16 leftLineTmpCnt,rightLineTmpCnt;//种子生长法所需参数

int16 leftLine[HEIGHT] = {0}, rightLine[HEIGHT] = {WIDTH - 1}, midLine[HEIGHT] = {0}, width[HEIGHT] = {0};//每个高度的左/右/中线及宽度

int16 regWidth[HEIGHT] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,29,36,39,51,53,56,64,67,73,79,89,97,100,105,110,116,121,129,137,143,146,153,159,166,171,176,182,188,194,200,206,212,217,223,229,235,239,245,251,257,263,271,277,283,289,295,301,307,313,318,324,330,336,341,347,353,359,365,371,377,383,389,395,401,407,413,419,425,431,437,443,449,455,461,467,473,479,485,491,497,};
//预处理的每行宽度（勉强能用）

uint8 leftExist[HEIGHT], rightExist[HEIGHT];//左线及右线的存在状态（如果要重构换成xxxline[]赋值MISS）
uint8 allFindCnt = 0,allLostCnt = 0, leftLostCnt = 0, rightLostCnt = 0;//左右线全找到计数，左右线全丢失计数，左线丢失计数，右线丢失计数
uint8 leftDownJump, rightDownJump, leftUpJump, rightUpJump, leftDownStart, rightDownStart, leftUpStart, rightUpStart;
// 左下/右下/左上/右上拐点          左下/右下/左上/右上边界存在起始点

RoadTypeEnum RoadType;//当前道路类型

int16 ThreeForkCnt = 0; // 0-未进入 1-进入1次,目前正在三岔内 2-进入2次,目前已经过一次 3-进入3次,目前正在三岔内 4-已完成三岔
                          //默认三岔左转

float parameterA, parameterB;//参数（或已废弃）

imgInfo img_info;//图像基础参数结构体（如果重构把上面以HEIGHT大小的开成结构体）


/**
 * @description: 图像处理调用函数
 * @param {*}
 * @return {*}
 */
void img_process(void)
{
	params_init();
    // RoadType = Cross;
	basic_searchLine();
    basic_getSpecialParams(img_info.top, img_info.bottom);

	road_judge();
    get_error();
}

/**
 * @description: 初始化每一幅图像的参数
 * @param {*}
 * @return {*}
 */
void params_init(void)
{
    leftLineTmpCnt = 0; rightLineTmpCnt = 0;
    allFindCnt = 0; allLostCnt = 0; leftLostCnt = 0; rightLostCnt = 0;
    leftDownJump = MISS; rightDownJump = MISS; leftUpJump = MISS; rightUpJump = MISS; leftDownStart = 0; rightDownStart = 0;
    leftUpStart = 0; rightUpStart = 0;
    memset(leftExist, 0, sizeof(leftExist));
    memset(rightExist, 0, sizeof(rightExist));
    // memset(leftLine, 0, sizeof(leftLine));
    // memset(rightLine, WIDTH - 1, sizeof(rightLine));
	img_info.top = 0;//UP_LIMIT;
	img_info.bottom = HEIGHT - 1;//DOWN_LIMIT;

    // for(int i = 1;)
}

/**
 * @description: 基础扫线函数
 *               获取图像的左右线存在与否，左右线坐标，有价值的图像范围等参数
 * @param {*}
 * @return {*}
 */
void basic_searchLine(void)
{
	/**********************************中线搜线*********************************/
	
	int m = WIDTH / 2, tmpTop=0;
	for(int i = img_info.bottom; i>img_info.top; --i)
	{
		int l, r;
		for(l = m; l>=0; --l)
		{
			if(!imageBin[i][l] && !imageBin[i][l+1])
			{
				leftLine[i] = l + 1;
				leftExist[i] = 1;
				break;
			}
		}
		for(r = m; r<WIDTH; ++r)
		{
			if(!imageBin[i][r] && !imageBin[i][r-1])
			{
				rightLine[i] = r - 1;
				rightExist[i] = 1;
				break;
			}
		} 
        if(!rightExist[i]) rightLine[i] = WIDTH;
		
		midLine[i] = (leftLine[i] + rightLine[i]) / 2;

		if (rightLine[i] < leftLine[i] ||( imageBin[i][midLine[i]] == 0 && imageBin[i - 1][midLine[i]] == 0))
		{
			tmpTop = i;
			
			if (tmpTop >= 20)    //防止在一开始就break
				break;
		}
        width[i] = rightLine[i] - leftLine[i];
        m = midLine[i];
	}

    img_info.top = tmpTop;

	/******************************************************************/
    //               TODO 种子生长法(under construction)
	/*******************************************************************/
	#if 1
	int dx[8] = {1, 1, 0,-1,-1,-1, 0, 1,};
	int dy[8] = {0,-1,-1,-1, 0, 1, 1, 1,};
    /**
     * 定义 # 周围为
     * 
     *   3 2 1
     *   4 # 0
     *   5 6 7
     * 
     */
	int16 start = HEIGHT - 1;
	int16 x, y, tx, ty, s, SearchCompleteFlag, SearchDisruptFlag;

    //计算起始行白点平均位置
    int whiteSum = 0, whiteCnt = 0, whiteAveMid;
    for(int i = 0; i < WIDTH; ++i)
    {
        if(imageBin[start][i])
        {
            whiteSum += i;
            ++whiteCnt;
        }
    }
    whiteAveMid = whiteSum / whiteCnt;
    
    //获取左右线起始位置
	leftLineSerial[leftLineTmpCnt].y = start;
	rightLineSerial[rightLineTmpCnt].y = start;
	for(int l = whiteAveMid; l >= 0; --l)
	{
		if(!imageBin[start][l] && !imageBin[start][l+1])
		{
			leftLineSerial[leftLineTmpCnt].x = l + 1;
            leftLine[start] = leftLineSerial[leftLineTmpCnt].x;
			leftExist[start] = 1;
		}
	}
	for(int r = whiteAveMid; r < WIDTH; ++r)
	{
		if(!imageBin[start][r] && !imageBin[start][r-1])
		{
			rightLineSerial[rightLineTmpCnt].x = r - 1;
            rightLine[start] = rightLineSerial[rightLineTmpCnt].x;
			rightExist[start] = 1;
		}
	}
    leftLineSerial[leftLineTmpCnt].y = start;
    rightLineSerial[rightLineTmpCnt].y = start;

    //种子生长法获得左线
	s = 0;
    x = leftLineSerial[leftLineTmpCnt].x;
    y = start;
    SearchCompleteFlag = 0;
    SearchDisruptFlag = 0;
    while(1)
    {
        //强制结束,防止扫过头判断条件
//        if()

        //逆时针旋转扫线
        for(int i = s; ; ++i)
        {
            if(i == 8) i = 0;
            tx = x + dx[i];
			ty = y + dy[i];

            if(ty < 0 || ty >= HEIGHT || tx >= WIDTH)//扫到边界,直接退出
            {
                SearchCompleteFlag = 1;
                break;
            }
			if(tx < 0)//断点,x=0
            {
                SearchDisruptFlag = 1;
                y = ty;
                break;
            }
            if(!imageBin[ty][tx])//黑色
            {
                x = tx;
                y = ty;
                break;
            }
        }

        if(SearchCompleteFlag == 1)
        {
            break;
        }
        if(SearchDisruptFlag == 1)
        {
            continue;
        }
        ++leftLineTmpCnt;
        leftLineSerial[leftLineTmpCnt].x = tx;
        leftLineSerial[leftLineTmpCnt].y = ty;
        leftExist[ty] = 1;
        if(!leftLine[ty]) leftLine[ty] = tx;
    }


	// while(!leftLine[y])//没有算过这个高度的值（如果后续要用完整的路径就另开个数组）
	// {
	// 	if( x == WIDTH || y == 0) break; //扫到头了
	// 	for(int i = s; ; ++i)//逆时针旋转
	// 	{        
	// 		if(i == 8) i = 0;
	// 		tx = x + dx[i];
	// 		ty = y + dy[i];
	// 		if(tx<0 || ty>=HEIGHT) break; // 如果越界了
	// 		if(tx == 0)
	// 		{
	// 			x = 0;
	// 			y = ty;
	// 			leftLine[ty] = tx;
	// 			break;
	// 		}
	// 		if(tx == WIDTH - 1)
	// 		{
	// 			x = WIDTH;
	// 			y = ty;
	// 			leftLine[ty] = tx;
	// 			break;
	// 		}
	// 		if(ty == 0)
	// 		{
	// 			x = tx;
	// 			y = 0;
	// 			leftLine[ty] = tx;
	// 			break;
	// 		}
	// 		if(!imageBin[tx][ty])
	// 		{
	// 			x = tx;
	// 			y = ty;
	// 			leftLine[ty] = tx;
	// 			leftExist[ty] = 1;
	// 			break;
	// 		}
	// 	}
	// 	s-=2;//倒回一个90度
	// }
	
	#endif
	
}

/**
 * @description: 获取拐点和起始点参数 (入十字及环岛拐点寻找所需)
 * @param {uint8} select_top
 * @param {uint8} select_bottom
 * @return {*}
 */
void basic_getSpecialParams(uint8 select_top, uint8 select_bottom)//TODO 基于种子生长法的拐点寻找
{
    int i;
    leftDownJump = MISS;
    leftUpJump = MISS;
    rightDownJump = MISS;
    rightUpJump = MISS;

    //寻找下方起点
    for (i = select_bottom; i >= select_top; --i)//获得左右线起始位置和缺失情况
    {
        //左线操作
        if(leftExist[i] && !leftDownStart)     //扫到线
        {
            leftDownStart = i;
        }
        //右线操作
        else if(rightExist[i] && !rightDownStart)    //扫到线
        {
            rightDownStart = i;
        }
    }
    
    // leftLastDiff = abs(leftLine[leftDownStart - 1] - leftLine[leftDownStart]);
    // rightLastDiff = abs(rightLine[rightDownStart - 1] - rightLine[rightDownStart]);
    //寻找下侧拐点
    for (i = leftDownStart - 1; i > select_top; --i)
    {
        //直接判断一个点的差值大于阈值，普适性不是很好
        if(leftLine[i - 1] - leftLine[i] < -3 && leftLine[i] - leftLine[i + 1] < 5)
        {
            leftDownJump = i;
            break;
        }
    }
    for (i = rightDownStart - 1 ; i > select_top; --i)
    {
        if(rightLine[i - 1] - rightLine[i] > 3 && rightLine[i + 1] - rightLine[i]  < 5)
        {
            rightDownJump = i;
            break;
        }
    }
    
    //寻找上侧起点
    for (i = select_top; i <= select_bottom; ++i)//获得左右线起始位置和缺失情况
    {
        //左线操作
        if(leftExist[i] && !leftUpStart)     //扫到线
        {
            leftUpStart = i;
        }  
        //右线操作
        if(rightExist[i] && !rightUpStart)    //扫到线
        {
            rightUpStart = i;
        }
    }
    //寻找上侧拐点
    for (i = leftUpStart + 2; i < select_bottom; ++i)//加2避开终止线干扰
    {
        if(leftLine[i + 1] - leftLine[i] < -5 && leftLine[i - 1] - leftLine[i] <= 5)
        {
            leftUpJump = i;
            break;
        }
    }
    for (i = rightUpStart + 2; i < select_bottom; ++i)
    {
        if(rightLine[i + 1] - rightLine[i] > 5 && rightLine[i] - rightLine[i - 1] <= 5)
        {
            rightUpJump = i;
            break;
        }
    }

    for(i = max(leftDownStart,rightDownStart); i > select_top; --i)
    {
        if (leftExist[i] == 1 && rightExist[i] == 1) allFindCnt++;//i <= 50 && 
        if (leftExist[i] == 0 && rightExist[i] == 0) allLostCnt++;//i <= 25 &&
        if(leftExist[i])
        {
            if(rightExist[i]) ++allFindCnt;
            else ++rightLostCnt;
        }
        else
        {
            if(rightExist[i]) ++leftLostCnt;
            else ++allLostCnt;
        }
    }
    return ;
}

/**
 * @description: 依赖下方拐点和起点修补下方缺失线的函数
 * @param {*}
 * @return {*}
 */
void basic_repairLine(void)//[x] 给予更多选项,道路下侧补线,midLine计算,[x]拆分出弯道判断模块)
{
    float k,b;
    uint8 leftDownEnd, rightDownEnd;

    if(leftDownJump != MISS)
        leftDownEnd = leftDownJump;
    else 
        leftDownEnd =  img_info.top;

    if(rightDownJump != MISS)
        rightDownEnd = rightDownJump;
    else
        rightDownEnd = img_info.top;
    //左线
    float left_curvature = get_curvature(leftDownStart, leftDownEnd,LEFT);
    if (left_curvature > -0.1 && left_curvature < 0.4 && leftDownStart - leftDownEnd >= 7)// && leftDownStart <= 32 )
    {
        least_squares(&k, &b, leftDownEnd, leftDownStart, LEFT);
        add_line(k, b, leftDownStart, img_info.bottom, LEFT);
    }

    //右线
    float right_curvature = get_curvature(rightDownStart, rightDownEnd, RIGHT);
    if (right_curvature > -0.4 && right_curvature < 0.1 && rightDownStart - rightDownEnd >= 7)// && rightDownStart <= 32 
    {
        least_squares(&k, &b, rightDownEnd, rightDownStart, RIGHT);
        add_line(k, b, rightDownStart, img_info.bottom, RIGHT);
    }

    //计算修补后的中线
    for(int i = img_info.bottom; i >= min(leftDownStart, rightDownStart); --i)
    {
        midLine[i] = (leftLine[i] + rightLine[i]) / 2;
        // width[i] = rightLine[i] - leftLine[i];
    }
    return ;
}
/**
 * @description: 根据选择范围的数据对应用范围补线,支持左中右线
 * @param {uint8} select_top
 * @param {uint8} select_bottom
 * @param {uint8} apply_top
 * @param {uint8} apply_bottom
 * @param {RoadTypeEnum} type
 * @return {*}
 */
void advance_repairLine(uint8 select_top, uint8 select_bottom, uint8 apply_top, uint8 apply_bottom, LineTypeEnum type)
{
    float k,b;

    switch (type)
    {
        case LEFT:
            least_squares(&k, &b, select_top, select_bottom, LEFT);
            add_line(k, b, apply_top, apply_bottom, LEFT);
            break;
        case RIGHT:
            least_squares(&k, &b, select_top, select_bottom, RIGHT);
            add_line(k, b, apply_top, apply_bottom, RIGHT);
            break;
        case MID:
            least_squares(&k, &b, select_top, select_bottom, MID);
            add_line(k, b, apply_top, apply_bottom, MID);
            break;
        default:
            break;
    }
}

/**
 * @description: 特判现在的道路元素
 * @param {*}
 * @return {*}
 */
void road_judge(void)
{
    /**
     * 
     * 思路是对的
     * 但目前拐点判断准确度不高
     * 所以没用。。
     *  
     * s弯使用拟合r值判断
     * 一边全丢是弯道
     * 一边双拐点 环岛
     * 
     */

    //旧状态机处理

    //十字入状态
    if(RoadType == Cross && leftUpJump != MISS && rightUpJump !=MISS)
    {
        #ifdef DEBUG
                std::cout<< "#In Cross#" <<std::endl;
        #endif
        float leftK, rightK, leftB, rightB;
        least_squares(&leftK, &leftB,max(img_info.top, leftUpJump - 7), leftUpJump, LEFT);
        least_squares(&rightK, &rightB,max(img_info.top,rightUpJump - 7),rightUpJump, RIGHT);

        add_line(leftK, leftB, leftUpJump, img_info.bottom, LEFT);
        add_line(rightK,rightB,rightUpJump,img_info.bottom,RIGHT);
        
        for(int i = img_info.bottom;i > min(leftUpJump, rightUpJump); --i)
        {
            midLine[i] = (leftLine[i] + rightLine[i]) / 2;
        }
        return ;
    }

    //新状态机启用
    

    //弯道
    if(allFindCnt <= 5 && allLostCnt <= 5)
    {
        if(leftLostCnt > rightLostCnt) //左边丢的比右边多,左弯
        {
            #ifdef DEBUG
                std::cout<< "#Turn Left#" <<std::endl;
            #endif
            RoadType = Turn_Left;
            turn_searchLine();
        }
        else //右边比左边多,右弯
        {
            #ifdef DEBUG
                std::cout<< "#Turn Right#" <<std::endl;
            #endif
            RoadType = Turn_Right;
            turn_searchLine();
        }
        return ;
    }

    //直道(必须在弯道后面判断)
    if((leftDownJump != MISS) + (rightDownJump != MISS) + (leftUpJump != MISS) + (rightUpJump != MISS) <= 1)
    {
        #ifdef DEBUG
                std::cout<< "#Straight#" <<std::endl;
        #endif
        RoadType = Straight;
        basic_repairLine();
        return ;
    }

    // 正入十字
    if(leftDownJump != MISS && leftUpJump != MISS && rightDownJump != MISS  && rightUpJump != MISS && RoadType != Three_Fork_Road)
    {
        #ifdef DEBUG
            std::cout<< "#Passing Cross Straightly#" <<std::endl;
        #endif
        RoadType = Cross;
        cross_searchLine();
        return ;
    }
    //斜入十字
    else if(leftDownJump != MISS && leftUpJump != MISS )
    {
        #ifdef DEBUG
            std::cout<< "#Passing Cross With Left#" <<std::endl;
        #endif
        RoadType = Cross;
        cross_searchLine_withLeft();
    }
    else if(rightDownJump != MISS && rightUpJump != MISS )
    {
        #ifdef DEBUG
            std::cout<< "#Passing Cross With Right#" <<std::endl;
        #endif
        RoadType = Cross;
        cross_searchLine_withRight();
    }

    return ;
}


/**
 * @description: 弯道特殊扫线
 * @param {*}
 * @return {*}
 */
void turn_searchLine(void)
{
    for(int i = max(leftDownStart,rightDownStart); i > img_info.top; --i)
    {
        if(leftExist[i])
        {
            if(rightExist[i])
                midLine[i] = (leftLine[i] + rightLine[i]) / 2;
            else
                midLine[i] = leftLine[i] + regWidth[i] / 2;
        }
        else
        {
            if(rightExist[i])
                midLine[i] = rightLine[i] - regWidth[i] / 2;
            else
                midLine[i] = midLine[i+1] + midLine[i+1] - midLine[i+2];
        }
    }
    return ;
}

/**
 * @description: 十字路口扫线
 * @param {*}
 * @return {*}
 */
void cross_searchLine(void)
{
    //左线
    float k = (float)(leftLine[leftUpJump] - leftLine[leftDownJump]) / (leftUpJump - leftDownJump);
    float b = (float)leftLine[leftDownJump] - k * leftDownJump;
    // leftLine[leftDownJump] = k * leftDownJump + b;

    add_line(k, b, leftUpJump, leftDownJump, LEFT);

    //右线
    k = (float)(rightLine[rightUpJump] - rightLine[rightDownJump]) / (rightUpJump - rightDownJump);
    b = (float)rightLine[rightDownJump] - k * rightDownJump;

    add_line(k, b, rightUpJump, rightDownJump, RIGHT);

    for(int i = min(leftUpJump, rightUpJump); i <= max(leftDownJump, rightDownJump); ++i)
    {
        midLine[i] = (leftLine[i] + rightLine[i]) / 2;
    }
    advance_repairLine(leftUpJump,leftDownStart,leftDownStart,img_info.bottom, LEFT);
    advance_repairLine(rightUpJump,rightDownStart,rightDownStart,img_info.bottom, RIGHT);
    for(int i = min(rightDownStart, leftDownStart); i <= img_info.bottom; ++i)
    {
        midLine[i] = (leftLine[i] + rightLine[i] ) / 2;
    }
    return ;
}
void cross_searchLine_withLeft(void)
{
    float leftK, leftB;
    leftK = (float)(leftLine[leftUpJump] - leftLine[leftDownJump]) / (leftUpJump - leftDownJump);
    leftB = (float)leftLine[leftUpJump] - leftK * leftUpJump;
    // least_squares(&rightK, &rightB, max(img_info.top,rightUpJump - 4), rightUpJump, RIGHT);
    add_line(leftK, leftB, leftUpJump, leftDownJump, LEFT);

    for(int i = img_info.bottom; i >= img_info.top; --i)
    {
        if(!rightExist[i])
        {
            midLine[i] = leftLine[i] + regWidth[i] / 2;
        }
        else if(!leftExist[i])
        {
            midLine[i] = rightLine[i] - regWidth[i] / 2;
        }
    }
    basic_repairLine();
    return ;
}
void cross_searchLine_withRight(void)
{
    float rightK, rightB;
    rightK = (float)(rightLine[rightUpJump] - rightLine[rightDownJump]) / (rightUpJump - rightDownJump);
    rightB = (float)rightLine[rightUpJump] - rightK * rightUpJump;
    // least_squares(&leftK, &leftB, max(img_info.top,leftUpJump - 4), leftUpJump, LEFT);
    add_line(rightK, rightB, rightUpJump, rightDownJump, RIGHT);
    
    for(int i = img_info.bottom; i >= img_info.top; --i)
    {
        if(!leftExist[i])
        {
            midLine[i] = rightLine[i] - regWidth[i] / 2;
        }
        else if(!rightExist[i])
        {
            midLine[i] = leftLine[i] + regWidth[i] / 2;
        }
    }
    basic_repairLine();
    return ;
}

/**
 * @description: 获取当前位置和中线位置的偏差，传递给PID处理
 * @param {*}
 * @return {*}
 */
void get_error(void)//TODO 实现动态调整前瞻
{
    int avePos, predictedPass;
    float spd = 2.5;// = get_speed() 之后写成由speed动态选择前瞻的模式
    predictedPass = min(spd * 2, img_info.top - 4); //之后根据实际情况写出关系式
    if (predictedPass < 0)
    {
        for(int i = img_info.bottom; i>img_info.top ; --i)
        {
            avePos += midLine[i];
        }
        avePos /= img_info.bottom - img_info.top;
    }
    else
    {
        avePos = (midLine[predictedPass] + midLine[predictedPass + 1] + midLine[predictedPass + 2]) / 3;
    }
    img_info.error = avePos-(WIDTH/2);
}
/**
 * @description: 判断左/中/右线的连续性
 * @param {uint8} select_top
 * @param {uint8} select_bottom
 * @param {LineTypeEnum} type
 * @return {*}
 */
uint8 judge_lineContinuity(uint8 select_top, uint8 select_bottom, LineTypeEnum type)
{
    int delta;
    switch (type)
    {
        case LEFT:
            for (int j = select_top; j < select_bottom; ++j)//从第10行开始，防止下面提早断掉，影响判断
            {
                delta = leftLine[j + 1] - leftLine[j];       //left线的偏差
                if (delta >= TH_ContinuityDelta || delta <= -TH_ContinuityDelta)
                {
                    return j;
                }
            }
            return 0;

            break;
        case MID:
            for (int j = select_top; j < select_bottom; ++j)//从第10行开始，防止下面提早断掉，影响判断
            {
                delta = midLine[j + 1] - midLine[j];       //left线的偏差
                if (delta >= TH_ContinuityDelta || delta <= -TH_ContinuityDelta)
                {
                    return j;
                }
            }
            return 0;

            break;
        case RIGHT:
            for (int j = select_top; j < select_bottom; ++j)//从第10行开始，防止下面提早断掉，影响判断
            {
                delta = rightLine[j + 1] - rightLine[j];       //left线的偏差
                if (delta >= TH_ContinuityDelta || delta <= -TH_ContinuityDelta)
                {
                    return j;
                }
            }
            return 0;

            break;

        default:
            break;
    }
    return 0;
}

float get_curvature(uint8 select_top, uint8 select_bottom, LineTypeEnum type)
{
    uint8 select_mid = (select_top + select_bottom) / 2;
    float ret;
    switch (type)
    {
        case LEFT:
            ret = calc_curvature(select_top, leftLine[select_top], 
                                      select_mid, leftLine[select_mid], 
                                      select_bottom, leftLine[select_bottom]);
            break;
        case RIGHT:
            ret = calc_curvature(select_top, rightLine[select_top], 
                                      select_mid, rightLine[select_mid], 
                                      select_bottom, rightLine[select_bottom]);
            break;
        case MID:
            ret = calc_curvature(select_top, midLine[select_top], 
                                      select_mid, midLine[select_mid], 
                                      select_bottom, midLine[select_bottom]);
            break;
        default:
            break;
    }
    return ret;
}

/**
 * @description: 三点计算曲率
 * @param {uint8} x1
 * @param {uint8} y1
 * @param {uint8} x2
 * @param {uint8} y2
 * @param {uint8} x3
 * @param {uint8} y3
 * @return {float}
 */
float calc_curvature(uint8 x1, uint8 y1, uint8 x2, uint8 y2, uint8 x3, uint8 y3)
{
    float K;
    int S_of_ABC = ((int)(x2 - x1) * (y3 - y1) - (int)(x3 - x1) * (y2 - y1)) / 2;
    //面积的符号表示方向
    int q1 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    float AB = sqrt((float)q1);
    q1 = (x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2);
    float BC = sqrt((float)q1);
    q1 = (x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1);
    float AC = sqrt((float)q1);
    if (AB * BC * AC <= 0.001)
        K = 0;
    else
        K = (float)4 * S_of_ABC / (AB * BC * AC);
    return K;
}


/**
 * @description: 补线（范围左开右闭）
 * @param {float} k 补线斜率 通过least_squares()函数获取
 * @param {float} b 补线偏移 通过least_squares()函数获取
 * @param {uint8} select_top 范围起点（不包括在内）
 * @param {uint8} select_bottom 范围终点（包括在内）
 * @param {LineTypeEnum} type 补线的对象类型 {LEFT/MID/RIGHT}
 * @return {*}
 */
void add_line(float k, float b, uint8 select_top, uint8 select_bottom, LineTypeEnum type)
{
    switch (type)
    {
        case LEFT:
            for(int i = select_top + 1; i <= select_bottom; ++i)
            {
                leftLine[i] = k * i + b;
            }
            break;
        case MID:
            for(int i = select_top + 1; i <= select_bottom; ++i)
            {
                midLine[i] = k * i + b;
            }
            break;
        case RIGHT:
            for(int i = select_top + 1; i <= select_bottom; ++i)
            {
                rightLine[i] = k * i + b;
            }
            break;
        default:
            break;
    }
}

/**
 * @description: 最小二乘法拟合直线
 *               注意：这里和一般图像的坐标系不一样，以左上角为原点，纵向为x_t，横向为y_t，
 *               即 y_t = $k$ * x_t + $b$ ，换成原坐标系直接带入当前图像高度 h_0 即可求出对应的 横坐标 w_0
 * @param {float *} k 传入直线斜率
 * @param {float *} b 传入直线偏移
 * @param {uint8} select_top 选定范围的开始（高度范围）
 * @param {uint8} select_bottom 选定范围的结束（高度范围）
 * @param {LeastSquaresObjectEnum} type 需要拟合的直线类型（左边界/右边界/中线）
 * @return {*} 无返回值，通过传入 $k$ 和 $b$ 得到直线参数
 */
void least_squares(float * k, float * b, uint8 select_top, uint8 select_bottom, LineTypeEnum type)
{
    int sumx = 0;                      /* sum of x     */
    int sumx2 = 0;                     /* sum of x**2  */
    int sumxy = 0;                     /* sum of x * y */
    int sumy = 0;                      /* sum of y     */
    int sumy2 = 0;                     /* sum of y**2  */
    int n = select_bottom - select_top + 1;

    switch(type) //传统最小二乘法无法拟合垂直的（而且越靠近越差），而我们遇到的垂直情况应该比较多，所以这里旋转一下坐标轴
    {
        case LEFT:
            for (int i = select_top; i <= select_bottom; ++i)
            {
                sumx  += i; //i 即为x轴值
                sumx2 += i * i;
                sumxy += i * leftLine[i];
                sumy  += leftLine[i];
                sumy2 += leftLine[i] * leftLine[i];
            }
            break;
        case MID:
            for (int i = select_top; i <= select_bottom; ++i)
            {
                sumx  += i; //i 即为x轴值
                sumx2 += i * i;
                sumxy += i * midLine[i];
                sumy  += midLine[i];
                sumy2 += midLine[i] * midLine[i];
            }
            break;
        case RIGHT:
            for (int i = select_top; i <= select_bottom; ++i)
            {
                sumx  += i; //i 即为x轴值
                sumx2 += i * i;
                sumxy += i * rightLine[i];
                sumy  += rightLine[i];
                sumy2 += rightLine[i] * rightLine[i];
            }
            break;

        default:
            break;
    }
    
    float denom = (n * sumx2 - sumx*sumx);
    if (denom == 0) {
        // singular matrix. can't solve the problem.(but we think it should be vertical to the x axis [doge])
        *k = 180;//给个比较大的数算了
        *b = 0;
        return ;
    }
    *k = (float)(n * sumxy  -  sumx * sumy) / denom;
    *b = (float)(sumy * sumx2  -  sumx * sumxy) / denom;
    // if (r!=NULL) {
    //     *r = (sumxy - sumx * sumy / n) /    /* compute correlation coeff */
    //           sqrt((sumx2 - sqr(sumx)/n) *
    //           (sumy2 - sqr(sumy)/n));
    // }
    return ;
}

/**
 * @description: 两段范围数据拟合直线（废弃状态）
 * @param {int} type
 * @param {int} startline1
 * @param {int} endline1
 * @param {int} startline2
 * @param {int} endline2
 * @return {*}
 */
void advanced_regression(int type, int startline1, int endline1, int startline2, int endline2)
{
    int i = 0;
    int sumlines1 = endline1 - startline1;
    int sumlines2 = endline2 - startline2;
    int sumX = 0;
    int sumY = 0;
    float averageX = 0;
    float averageY = 0;
    float sumUp = 0;
    float sumDown = 0;
    if (type == 0)  //拟合中线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += midLine[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += midLine[i];
        }
        averageX = (float)sumX / (sumlines1 + sumlines2);     //x的平均值
        averageY = (float)sumY / (sumlines1 + sumlines2);     //y的平均值
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (midLine[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (midLine[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown <= 1e-5) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;

    }
    else if (type == 1)     //拟合左线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += leftLine[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += leftLine[i];
        }
        averageX = (float)sumX / (sumlines1 + sumlines2);     //x的平均值
        averageY = (float)sumY / (sumlines1 + sumlines2);     //y的平均值
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (leftLine[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (leftLine[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown <= 1e-5) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 2)         //拟合右线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += rightLine[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += rightLine[i];
        }
        averageX = (float)sumX / (sumlines1 + sumlines2);     //x的平均值
        averageY = (float)sumY / (sumlines1 + sumlines2);     //y的平均值
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (rightLine[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (rightLine[i] - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
}
