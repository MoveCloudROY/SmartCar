/*
 * @Author: ROY1994
 * @Date: 2022-02-04 14:01:30
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-04-14 20:56:37
 * @FilePath: \myImageDeal\rowInfo.cpp
 * @Description: 搜线，元素判断等主要处理函数
 */
#include "ImageDeal.h"


int LineEdgeScanWindow_Cross = 3;  //十字扫线范围
int LineEdgeScanWindow = 5;    //直道方差阈值

uint8_t needExternL = 0;  //是否左延长标志
uint8_t needExternR = 0;  //是否右延长标志

extern uint8_t mt9v03x_image[120][188];//原灰度图
extern uint8_t imageBin[HEIGHT][WIDTH];//二值化图像
extern ConstDataTypeDef ConstData;

PixelTypedef leftLineSerial[HEIGHT<<1],rightLineSerial[HEIGHT<<1];//种子生长法所需参数
int16_t leftLineSeriesCnt,rightLineSeriesCnt;//种子生长法所需参数

RowInfoTypedef rowInfo[HEIGHT];//行信息

int16_t regWidth[HEIGHT] = {0,0,0,0,0,0,0,0,0,0,0,28,28,28,30,31,33,33,35,36,37,38,39,40,42,42,44,46,46,48,49,50,52,52,54,54,56,58,58,60,61,62,64,64,66,67,68,69,71,71,73,74,75,77,79,79,80,81,83,84,85,86,88,88,90,91,93,94,95,96,97,98,99,101,102,103,105,105,107,107,109,110,111,113,114,115,115,117,117,119,119,121,122,123,124,125,127,127,129,130,131,133,133,135,136,137,139,139,141,141,143,144,145,146,148,149,150,151,152,153,};
//预处理的每行宽度（勉强能用）

//左线及右线的存在状态（如果要重构换成xxxline[]赋值MISS）
uint8_t allFindCnt = 0,allLostCnt = 0, leftLostCnt = 0, rightLostCnt = 0;//左右线全找到计数，左右线全丢失计数，左线丢失计数，右线丢失计数
uint8_t leftDownJump, rightDownJump, leftUpJump, rightUpJump, leftDownStart, rightDownStart, leftUpStart, rightUpStart;
// 左下/右下/左上/右上拐点          左下/右下/左上/右上边界存在起始点
uint8_t leftSeriesBreak, rightSeriesBreak;


int16_t ThreeForkCnt = 0; // 0-未进入 1-进入1次,目前正在三岔内 2-进入2次,目前已经过一次 3-进入3次,目前正在三岔内 4-已完成三岔
                          //默认三岔左转

float parameterA, parameterB;//参数（或已废弃）

ImgInfoTypedef imgInfo;//图像基础参数结构体（如果重构把上面以HEIGHT大小的开成结构体）


uint8_t ForkLinePointx_l, ForkLinePointx_r, ForkLinePointy;//三岔点补线坐标
int fork_spoint_1_Y = 0, fork_spoint_2_Y = 0;
char fork_flag_1 = 0, fork_flag_2 = 0, fork_flag_tot = 0;

/**
 * @description: 图像处理调用函数
 * @param {*}
 * @return {*}
 */
void img_process(void)
{
    params_init();
    // RoadType = Cross;
    basic_searchLine(HEIGHT-1,HEIGHT-6);
    advance_searchLine(HEIGHT-7);
    advance_repairLine();

//    imgInfo.CircleStatus = CIRCLE_OUT;
//    imgInfo.RoadType = Circle_L;

    road_judge();
    advance_midLineFilter();

    //series_sea-rchLine();
    //basic_getSpecialParams(imgInfo.top, imgInfo.bottom);
    // series_getSpecialParams();


    get_error();
}

/**
 * @description: 初始化每一幅图像的参数
 * @param {*}
 * @return {*}
 */
void params_init(void)
{
    leftLineSeriesCnt = 0; rightLineSeriesCnt = 0;
    allFindCnt = 0; allLostCnt = 0; leftLostCnt = 0; rightLostCnt = 0;
    leftDownJump = MISS; rightDownJump = MISS; leftUpJump = MISS; rightUpJump = MISS; leftDownStart = 0; rightDownStart = 0;
    leftUpStart = 0; rightUpStart = 0;
    // memset(rowInfo, 0, sizeof(rowInfo));
    // memset(rowInfo, 0, sizeof(rowInfo));
    // memset(leftLine, 0, sizeof(leftLine));
    // memset(rowInfo, WIDTH - 1, sizeof(rowInfo));
    ForkLinePointx_l = 0; ForkLinePointx_r = 0; ForkLinePointy = 0;
    fork_flag_1 = 'F';
    fork_flag_2 = 'F';
    imgInfo.top = 8;//UP_LIMIT;
    imgInfo.bottom = HEIGHT - 1;//DOWN_LIMIT;
    for(int i = 0; i  < HEIGHT; ++i)
    {
        rowInfo[i].leftStatus = LOST;
        rowInfo[i].rightStatus = LOST;
        rowInfo[i].leftLine = 0;
        rowInfo[i].rightLine = WIDTH - 1;
        rowInfo[i].fork_L = MISS;
        rowInfo[i].fork_R = MISS;
        rowInfo[i].fork_blackWidth = 0;
    }

    // for(int i = 1;)
}

/**
 * @description: 基础扫线函数(前n行)
 *               获取图像的左右线存在与否，左右线坐标，有价值的图像范围等参数
 * @param {*}
 * @return {*}
 */
void basic_searchLine(int bottom,int top)
{
    /**********************************中线搜线*********************************/

    int whiteSum = 0, whiteCnt = 0, whiteAveMid;
    for(int i = 0; i < WIDTH; ++i)
    {
        if(imageBin[bottom][i])
        {
            whiteSum += i;
            ++whiteCnt;
        }
    }
    whiteAveMid = whiteSum / whiteCnt;
    int m = whiteAveMid;
    for(int i = bottom; i >= top; --i)
    {
        int l, r;
        for(l = m; l>=0; --l)
        {
            if(!imageBin[i][l] && !imageBin[i][l+1])
            {
                rowInfo[i].leftLine = l + 1;
                rowInfo[i].leftStatus = EXIST;
                break;
            }
        }
        for(r = m; r<WIDTH; ++r)
        {
            if(!imageBin[i][r] && !imageBin[i][r-1])
            {
                rowInfo[i].rightLine = r - 1;
                rowInfo[i].rightStatus = EXIST;
                break;
            }
        }
        if(rowInfo[i].rightStatus != EXIST)
        {
            rowInfo[i].rightLine= WIDTH - 1;
            rowInfo[i].rightStatus = EXIST;
        }
        if(rowInfo[i].leftStatus != EXIST)
        {
            rowInfo[i].leftLine = 0;
            rowInfo[i].leftStatus = EXIST;
        }

        rowInfo[i].midLine = (rowInfo[i].leftLine + rowInfo[i].rightLine) / 2;
        rowInfo[i].width = rowInfo[i].rightLine - rowInfo[i].leftLine;
        m = rowInfo[i].midLine;

    }
}
/**
 * @description: 八邻域扫线
 * @param {*}
 * @return {*}
 */
void series_searchLine(void)
{
    /******************************************************************/
    //               TODO 种子生长法(under construction)
    /*******************************************************************/
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
    int16_t start = HEIGHT - 1;
    int16_t x, y, tx, ty, s, SearchCompleteFlag, SearchDisruptFlag;

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
    ++leftLineSeriesCnt;
    ++rightLineSeriesCnt;
    leftLineSerial[leftLineSeriesCnt].x = 0;
    rightLineSerial[rightLineSeriesCnt].x = WIDTH - 1;
    leftLineSerial[leftLineSeriesCnt].y = start;
    rightLineSerial[rightLineSeriesCnt].y = start;
    for(int l = whiteAveMid; l >= 0; --l)
    {
        if(!imageBin[start][l] && !imageBin[start][l+1])
        {
            leftLineSerial[leftLineSeriesCnt].x = l + 1;
            break;
        }
    }
    for(int r = whiteAveMid; r < WIDTH; ++r)
    {
        if(!imageBin[start][r] && !imageBin[start][r-1])
        {
            rightLineSerial[rightLineSeriesCnt].x = r - 1;
            break;
        }
    }

    /*-------------------种子生长法获得左线-------------------*/
    s = 0;
    x = leftLineSerial[leftLineSeriesCnt].x;
    y = start;
    SearchCompleteFlag = 0; // 搜索完成标志位
    SearchDisruptFlag = 0;  // 搜索中途空行标志位

    while(1)
    {
        //强制结束,防止扫过头
        if(y <= imgInfo.top)
            break;
        // printf(" $%d %d$ \n", x, y);
        //逆时针旋转扫线
        for( ; ; ++s)
        {
            s = (s + 8) % 8;
            // if(s == 8) s = 0;

            tx = x + dx[s];// 周围元素的坐标
            ty = y + dy[s];
            if(ty < 0 || ty >= HEIGHT || tx >= WIDTH)// 扫到边界，搜索完成
            {
                SearchCompleteFlag = 1;
                break;
            }
            if(tx < 0)// 在空行内，上次的x值不变，ty继续移动
            {
                SearchDisruptFlag = 1;
                y = ty;
                s =  (s - 1 + 8) % 8;
                break;
            }
            if(!imageBin[ty][tx])// 黑色（左线跳变）更新x和y
            {
                SearchDisruptFlag = 0;//退出空行状态
                x = tx;
                y = ty;
                s =  (s - 2 + 8) % 8;
                break;
            }
        }

        if(SearchCompleteFlag == 1)// 搜索完成退出
        {
            break;
        }
        if(SearchDisruptFlag == 1)// 遇到中断继续
        {
            continue;// TODO 加入分隔符
        }

        ++leftLineSeriesCnt;// 继续增加当前线的长度
        leftLineSerial[leftLineSeriesCnt].x = tx;
        leftLineSerial[leftLineSeriesCnt].y = ty;
    }

    /*-------------------种子生长法获得右线-------------------*/
    //注意，此时应当换为顺时针方向搜索
    s = 4;
    x = rightLineSerial[rightLineSeriesCnt].x;
    y = start;
    SearchCompleteFlag = 0; // 搜索完成标志位
    SearchDisruptFlag = 0;  // 搜索中途空行标志位

    while(1)
    {
        //强制结束,防止扫过头
        if(y <= imgInfo.top)
            break;
        // printf(" $%d %d$ \n", x, y);
        //顺时针旋转扫线
        for( ; ; --s)
        {
            s = (s + 8) % 8;
            // if(s == 8) s = 0;

            tx = x + dx[s];// 周围元素的坐标
            ty = y + dy[s];
            if(ty < 0 || ty >= HEIGHT || tx < 0)// 扫到边界，搜索完成
            {
                SearchCompleteFlag = 1;
                break;
            }
            if(tx >= WIDTH)// 在空行内，上次的x值不变，ty继续移动
            {
                SearchDisruptFlag = 1;
                y = ty;
                s =  (s + 1 + 8) % 8;
                break;
            }
            if(!imageBin[ty][tx])// 黑色（右线跳变）更新x和y
            {
                SearchDisruptFlag = 0;
                x = tx;
                y = ty;
                s =  (s + 2 + 8) % 8;
                break;
            }
        }

        if(SearchCompleteFlag == 1)// 搜索完成退出
        {
            break;
        }
        if(SearchDisruptFlag == 1)// 遇到中断继续
        {
            continue;// TODO 加入分隔符
        }

        ++rightLineSeriesCnt;// 继续增加当前线的长度
        rightLineSerial[rightLineSeriesCnt].x = tx;
        rightLineSerial[rightLineSeriesCnt].y = ty;
    }
}
/**
 * @description: 获取拐点和起始点参数 (入十字及环岛拐点寻找所需)
 * @param {uint8_t} select_top
 * @param {uint8_t} select_bottom
 * @return {*}
 */
void basic_getSpecialParams(uint8_t select_top, uint8_t select_bottom)//TODO 基于种子生长法的拐点寻找
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
        if(rowInfo[i].leftStatus == EXIST && !leftDownStart)     //扫到线
        {
            leftDownStart = i;
        }
        //右线操作
        else if(rowInfo[i].rightStatus == EXIST && !rightDownStart)    //扫到线
        {
            rightDownStart = i;
        }
    }
    
    // leftLastDiff = abs(leftLine[leftDownStart - 1] - leftLine[leftDownStart]);
    // rightLastDiff = abs(rowInfo[rightDownStart - 1] - rowInfo[rightDownStart]);
    //寻找下侧拐点
    for (i = leftDownStart - 1; i > select_top; --i)
    {
        //直接判断一个点的差值大于阈值，普适性不是很好
        if(rowInfo[i - 1].leftLine - rowInfo[i].leftLine < -3 && rowInfo[i].leftLine - rowInfo[i + 1].leftLine < 5)
        {
            leftDownJump = i;
            break;
        }
    }
    for (i = rightDownStart - 1 ; i > select_top; --i)
    {
        if(rowInfo[i - 1].rightLine - rowInfo[i].rightLine > 3 && rowInfo[i + 1].rightLine - rowInfo[i].rightLine  < 5)
        {
            rightDownJump = i;
            break;
        }
    }
    
    //寻找上侧起点
    for (i = select_top; i <= select_bottom; ++i)//获得左右线起始位置和缺失情况
    {
        //左线操作
        if(rowInfo[i].leftStatus == EXIST && !leftUpStart)     //扫到线
        {
            leftUpStart = i;
        }  
        //右线操作
        if(rowInfo[i].rightStatus == EXIST && !rightUpStart)    //扫到线
        {
            rightUpStart = i;
        }
    }
    //寻找上侧拐点
    for (i = leftUpStart + 2; i < select_bottom; ++i)//加2避开终止线干扰
    {
        if(rowInfo[i + 1].leftLine - rowInfo[i].leftLine < -5 && rowInfo[i - 1].leftLine - rowInfo[i].leftLine <= 5)
        {
            leftUpJump = i;
            break;
        }
    }
    for (i = rightUpStart + 2; i < select_bottom; ++i)
    {
        if(rowInfo[i + 1].rightLine - rowInfo[i].rightLine > 5 && rowInfo[i].rightLine - rowInfo[i - 1].rightLine <= 5)
        {
            rightUpJump = i;
            break;
        }
    }

    for(i = max(leftDownStart,rightDownStart); i > select_top; --i)
    {
        // if (rowInfo[i].leftStatus == EXIST && rowInfo[i].rightStatus == EXIST) allFindCnt++;//i <= 50 &&
        // if (rowInfo[i].leftStatus != EXIST && rowInfo[i].rightStatus != EXIST) allLostCnt++;//i <= 25 &&
        if(rowInfo[i].leftStatus == EXIST)
        {
            if(rowInfo[i].rightStatus == EXIST) ++allFindCnt;
            else ++rightLostCnt;
        }
        else
        {
            if(rowInfo[i].rightStatus == EXIST) ++leftLostCnt;
            else ++allLostCnt;
        }
    }
    return ;
}


void series_getSpecialParams(void)
{
    //起始点直接取就完事了
    leftDownStart = leftLineSerial[1].y;
    rightDownStart = rightLineSerial[1].y;
    leftUpStart = leftLineSerial[leftLineSeriesCnt].y;
    rightUpStart = rightLineSerial[rightLineSeriesCnt].y;

    //默认情况下断行位于最底下
    leftSeriesBreak = leftLineSeriesCnt;
    rightSeriesBreak = rightLineSeriesCnt;

    //prev4为左侧第4个元素, next4为右侧第四个元素, now为当前元素, leftMaxCosi rightMaxCosi 为夹角最大余弦值对应的元素
    int prev4 = 1, next4 = 1, leftMaxCosi, rightMaxCosi;
    //p1 p2 为 两个向量模长, cosTmp 为余弦值, leftMaxCos rightMaxCos 为夹角最大余弦值
    float cosTmp, leftMaxCos = -1.1, rightMaxCos = -1.1;


    // 左线
    for (int i = 1; i <= leftLineSeriesCnt-4; ++i)
    {
        // 考虑线的连续情况，若连续，则该点前后两个元素的x，y差应该在1以内
        // 如果非连续则可推出：
        //      1. 存在空行
        //      2. 可能存在上拐点

        if(i <= leftSeriesBreak)// 在断行之下
        {
            //移动prev4和next4
            prev4 = max(1,i-4);
            next4 = min(leftSeriesBreak,i+4);
            //断行判断
            if(abs(leftLineSerial[next4].x - leftLineSerial[next4 + 1].x) > 1 ||
            abs(leftLineSerial[next4].y - leftLineSerial[next4 + 1].y) > 1)
            {
                leftSeriesBreak = next4;
            }

            //计算此刻余弦值
            cosTmp = cosAOB(leftLineSerial[prev4].x, leftLineSerial[prev4].y,
                            leftLineSerial[i].x, leftLineSerial[i].y,
                            leftLineSerial[next4].x, leftLineSerial[next4].y);
            //更新
            if(cosTmp > leftMaxCos)
            {
                leftMaxCosi = i;
                leftMaxCos = cosTmp;
            }
        }
        else//在断行之上
        {
            //第一次到断行上, 要复位max值, 顺带保存拐点
            if(i == leftSeriesBreak + 1)
            {
                leftDownJump = leftLineSerial[leftMaxCosi].y;

                leftMaxCos = -1.1;
            }
            //移动prev4和next4
            prev4 = max(leftSeriesBreak+1,i-4);
            next4 = min(leftLineSeriesCnt,i+4);
            //计算此刻余弦值
            cosTmp = cosAOB(leftLineSerial[prev4].x, leftLineSerial[prev4].y,
                            leftLineSerial[i].x, leftLineSerial[i].y,
                            leftLineSerial[next4].x, leftLineSerial[next4].y);
            //更新
            if(cosTmp > leftMaxCos)
            {
                leftMaxCosi = i;
                leftMaxCos = cosTmp;
                // printf("LD %d: %f\n",i, leftMaxCos);
            }
        }

    }
    //如果leftSeriesBreak出现变更, 考虑是否存在上拐点
    if(leftSeriesBreak != leftLineSeriesCnt)
    {
        leftUpJump = leftLineSerial[leftMaxCosi].y;
        // printf("LU: %f\n", leftMaxCos);
    }

    // 右线
    prev4 = 1; next4 = 1;
    for (int i = 1; i <= rightLineSeriesCnt-4; ++i)
    {
        // 考虑线的连续情况，若连续，则该点前后两个元素的x，y差应该在1以内
        // 如果非连续则可推出：
        //      1. 存在空行
        //      2. 可能存在上拐点

        if(i <= rightSeriesBreak)// 在断行之下
        {
            //移动prev4和next4
            prev4 = max(1,i-4);
            next4 = min(rightSeriesBreak,i+4);
            //断行判断
            if(abs(rightLineSerial[next4].x - rightLineSerial[next4 + 1].x) > 1 ||
            abs(rightLineSerial[next4].y - rightLineSerial[next4 + 1].y) > 1)
            {
                rightSeriesBreak = next4;
            }

            //计算此刻余弦值
            cosTmp = cosAOB(rightLineSerial[prev4].x, rightLineSerial[prev4].y,
                            rightLineSerial[i].x, rightLineSerial[i].y,
                            rightLineSerial[next4].x, rightLineSerial[next4].y);
            //更新
            if(cosTmp > rightMaxCos)
            {
                rightMaxCosi = i;
                rightMaxCos = cosTmp;
            }
        }
        else//在断行之上
        {
            //第一次到断行上, 要复位max值, 顺带保存拐点
            if(i == rightSeriesBreak + 1)
            {
                rightDownJump = rightLineSerial[rightMaxCosi].y;
                rightMaxCos = -1.1;
            }
            //移动prev4和next4
            prev4 = max(rightSeriesBreak+1,i-4);
            next4 = min(rightLineSeriesCnt,i+4);
            //计算此刻余弦值
            cosTmp = cosAOB(rightLineSerial[prev4].x, rightLineSerial[prev4].y,
                            rightLineSerial[i].x, rightLineSerial[i].y,
                            rightLineSerial[next4].x, rightLineSerial[next4].y);
            //更新
            if(cosTmp > rightMaxCos)
            {
                rightMaxCosi = i;
                rightMaxCos = cosTmp;
            }
        }

    }
    //如果rightSeriesBreak出现变更, 考虑是否存在上拐点
    if(rightSeriesBreak != rightLineSeriesCnt)
    {
        rightUpJump = rightLineSerial[rightMaxCosi].y;
    }
}

/**
 * @description: 依赖下方拐点和起点修补下方缺失线的函数
 * @param {*}
 * @return {*}
 */
void basic_repairLine(void)//[x] 给予更多选项,道路下侧补线,midLine计算,[x]拆分出弯道判断模块)
{
    float k,b;
    uint8_t leftDownEnd, rightDownEnd;

    if(leftDownJump != MISS)
        leftDownEnd = leftDownJump;
    else 
        leftDownEnd =  imgInfo.top;

    if(rightDownJump != MISS)
        rightDownEnd = rightDownJump;
    else
        rightDownEnd = imgInfo.top;
    //左线
    float left_curvature = get_curvature(leftDownStart, leftDownEnd,LEFT);
    if (left_curvature > -0.1 && left_curvature < 0.4 && leftDownStart - leftDownEnd >= 7)// && leftDownStart <= 32 )
    {
        least_squares(&k, &b, leftDownEnd, leftDownStart, LEFT);
        add_line(k, b, leftDownStart, imgInfo.bottom, LEFT);
    }

    //右线
    float right_curvature = get_curvature(rightDownStart, rightDownEnd, RIGHT);
    if (right_curvature > -0.4 && right_curvature < 0.1 && rightDownStart - rightDownEnd >= 7)// && rightDownStart <= 32 
    {
        least_squares(&k, &b, rightDownEnd, rightDownStart, RIGHT);
        add_line(k, b, rightDownStart, imgInfo.bottom, RIGHT);
    }

    //计算修补后的中线
    for(int i = imgInfo.bottom; i >= min(leftDownStart, rightDownStart); --i)
    {
        rowInfo[i].midLine = (rowInfo[i].leftLine + rowInfo[i].rightLine) / 2;
        // rowInfo[i] = rowInfo[i] - leftLine[i];
    }
    return ;
}
#define LIMIT_L(x) (x = ((x) < 1? 1 : (x)))
#define LIMIT_R(x) (x = ((x) > WIDTH - 2? WIDTH - 2 : (x)))
#define LIMIT_H(x) ((x) >= HEIGHT ? HEIGHT - 1: (x))
void advance_searchLine(int bottom)  //////不用更改
{
    uint8_t isFindLk = 'F';     //确定无边斜率的基准有边行是否被找到的标志
    uint8_t hasFindLk = 'F';    //是否尝试过找到这一帧图像的基准左斜率
    uint8_t isFindRk = 'F';     //确定无边斜率的基准有边行是否被找到的标志
    uint8_t hasFindRk = 'F';    //是否尝试过找到这一帧图像的基准右斜率
    float D_L = 0;              //延长线左边线斜率
    float D_R = 0;              //延长线右边线斜率
    int firstLostL = 0;             //记住首次左丢边行
    int firstLostR = 0;             //记住首次右丢边行
    needExternR = 0;            //标志位清0
    needExternL = 0;
    int tmpL = 0, tmpR = 0;
    for (int row = bottom; row > imgInfo.top; row--)  //前5行处理过了，下面从55行到（设定的不处理的行top）
    {   //太远的图像不稳定，top以后的不处理
        // picTemp = imageBin[row];

        EdgePointTypedef EdgePoint[2];  // 0左1右
        if (imgInfo.RoadType != Cross)
        {
            tmpL = (int)rowInfo[row + 1].rightLine - LineEdgeScanWindow;  //从上一行右边线-5的点开始（确定扫描开始点）
            tmpR = (int)rowInfo[row + 1].rightLine + LineEdgeScanWindow;  //到上一行右边线+5的点结束（确定扫描结束点）
        }
        else
        {
            tmpL = (int)rowInfo[row + 1].rightLine - LineEdgeScanWindow_Cross;  //从上一行右边线-5的点开始（确定扫描开始点）
            tmpR = (int)rowInfo[row + 1].rightLine + LineEdgeScanWindow_Cross;
        }

        LIMIT_L(tmpL);   //确定左扫描区间并进行限制
        LIMIT_R(tmpR);  //确定右扫描区间并进行限制


        basic_getJumpPointFromDet(imageBin[row], tmpL, tmpR, &EdgePoint[1], RIGHT);  //扫右边线

        tmpL = (int)rowInfo[row + 1].leftLine - LineEdgeScanWindow;  //从上一行左边线-5的点开始（确定扫描开始点）
        tmpR = (int)rowInfo[row + 1].leftLine + LineEdgeScanWindow;  //到上一行左边线+5的点结束（确定扫描结束点）

        LIMIT_L(tmpL);   //确定左扫描区间并进行限制
        LIMIT_R(tmpR);  //确定右扫描区间并进行限制


        basic_getJumpPointFromDet(imageBin[row],tmpL, tmpR, &EdgePoint[0], LEFT);

        if (EdgePoint[0].type == LOST)  //如果本行左边线不正常跳变，即这10个点都是白的
        {
            rowInfo[row].leftLine = rowInfo[row + 1].leftLine;  //本行左边线用上一行的数值
        }
        else //左边线正常
        {
            rowInfo[row].leftLine = EdgePoint[0].posX;  //记录下来啦
        }

        if (EdgePoint[1].type == LOST)  //如果本行右边线不正常跳变
        {
            rowInfo[row].rightLine = rowInfo[row + 1].rightLine;  //本行右边线用上一行的数值
        }
        else //右边线正常
        {
            rowInfo[row].rightLine = EdgePoint[1].posX;  //记录下来啦
        }

        rowInfo[row].leftStatus = EdgePoint[0].type;  //记录本行是否找到边线，即边线类型
        rowInfo[row].rightStatus = EdgePoint[1].type;

        //重新确定那些大跳变的边缘
        if (EdgePoint[0].type == JUMP || EdgePoint[1].type == JUMP)
        {
            if (EdgePoint[0].type == JUMP )  //如果左边线大跳变
            {
                for (int col = (rowInfo[row].leftLine + 1); col <= (rowInfo[row].rightLine - 1); col++)  //左右边线之间重新扫描
                {
                    if (*(imageBin[row] + col) == 0 && *(imageBin[row] + col + 1) != 0)
                    {
                        rowInfo[row].leftLine = col;  //如果上一行左边线的右边有黑白跳变则为绝对边线直接取出
                        rowInfo[row].leftStatus = EXIST;
                        break;
                    }
                    else if (*(imageBin[row] + col) != 0)  ///一旦出现白点则直接跳出
                        break;
                    else if (col == (rowInfo[row].rightLine - 1))
                    {  //如果到了右边线的最右边，则认为是绝对边线
                        rowInfo[row].leftLine = col;
                        rowInfo[row].leftStatus = EXIST;
                        break;
                    }
                }
            }
            if ((rowInfo[row].rightLine - rowInfo[row].leftLine) <= 7)  //图像宽度限定
            {
                imgInfo.top = row + 1;  //如果这行比7小了后面直接不要了
                break;
            }
            if (EdgePoint[1].type == JUMP)
                for (int col = (rowInfo[row].rightLine - 1); col >= (rowInfo[row].leftLine + 1); col--)
                {
                    if ((*(imageBin[row] + col) == 0) && (*(imageBin[row] + col - 1) != 0))
                    {
                        rowInfo[row].rightLine = col;  ////如果右边线的左边还有黑白跳变则为绝对边线直接取出
                        rowInfo[row].rightStatus = EXIST;
                        break;
                    }
                    else if (*(imageBin[row] + col) != 0)
                        break;
                    else if (col == (rowInfo[row].leftLine + 1))
                    {  //如果到了左边线的最左边，则认为是绝对边线
                        rowInfo[row].rightLine = col;
                        rowInfo[row].rightStatus = EXIST;
                        break;
                    }
                }
        }

        //        /***********重新确定无边行************/
        uint8_t L_found_point = 0;
        uint8_t R_found_point = 0;
        int tmpRightLine = WIDTH - 2, tmpLeftLine = 1;

        if (EdgePoint[1].type == LOST && row > 10 && row < HEIGHT-10) //最早出现的无边行
        {
            if (hasFindRk == 'F') //这一帧图像没有跑过这个找基准线循环
            {
                hasFindRk = 'T';      //找了  一帧图像只跑一次 置为T
                firstLostR = row + 2; //
                for (int y = row + 1; y < LIMIT_H(row + 30); y++)
                {
                    if (rowInfo[y].rightStatus == EXIST) //往无边行下面搜索  一般都是有边的
                        R_found_point++;
                }
                if (R_found_point > 8) //找到基准斜率边  做延长线重新确定无边   当有边的点数大于8
                {
                    D_R = ((float)(rowInfo[row + R_found_point].rightLine - rowInfo[row + 3].rightLine))
                        / ((float)(R_found_point - 3));
                                                        //求下面这些点连起来的斜率
                                                        //好给无边行做延长线左个基准
                    if (D_R >= 0)
                    {
                        isFindRk ='T';
                            //如果斜率大于0  那么找到了这个基准行  因为梯形畸变
                            //所以一般情况都是斜率大于0 小于0的情况也不用延长 没必要
                    }
                    else
                    {
                        isFindRk = 'F'; //没有找到这个基准行
                        if (D_R < 0)
                            needExternR = 'F'; //这个标志位用于十字角点补线  防止图像误补用的
                    }
                }
            }
            if (isFindRk == 'T')
            {
                tmpRightLine = rowInfo[firstLostR].rightLine - D_R * (firstLostR - row); //如果找到了 那么以基准行做延长线
            }

            LIMIT_L(tmpRightLine); //限幅
            LIMIT_R(tmpRightLine); //限幅
            rowInfo[row].rightLine = tmpRightLine;

        }

        if (EdgePoint[0].type == LOST && row > 10 && row < HEIGHT-10) //下面同理  左边界
        {
            if (hasFindLk == 'F')
            {
                hasFindLk = 'T';
                firstLostL = row + 2;
                for (int y = row + 1; y < LIMIT_H(row + 30); y++)
                {
                    if (rowInfo[y].leftStatus == EXIST)
                        L_found_point++;
                }
                if (L_found_point > 8) //找到基准斜率边  做延长线重新确定无边
                {
                    D_L = ((float)(rowInfo[row + 3].leftLine - rowInfo[row + L_found_point].leftLine))
                        / ((float)(L_found_point - 3));

                    if (D_L > 0)
                    {
                        isFindLk = 'T';
                    }
                    else
                    {
                        isFindLk = 'F';
                        if (D_L < 0)
                            needExternL = 'F';
                    }
                }
            }

            if (isFindLk == 'T')
                tmpLeftLine = rowInfo[firstLostL].leftLine + D_L * (firstLostL - row);

            LIMIT_L(tmpLeftLine); //限幅
            LIMIT_R(tmpLeftLine); //限幅
            rowInfo[row].leftLine = tmpLeftLine;
        }

        // if (rowInfo[row].leftStatus == LOST && rowInfo[row].rightStatus == LOST)
        //         imgInfo.allLostCnt++; //要是左右都无边，丢边数+1

        LIMIT_L(rowInfo[row].leftLine);  //限幅
        LIMIT_R(rowInfo[row].leftLine);  //限幅
        LIMIT_L(rowInfo[row].rightLine); //限幅
        LIMIT_R(rowInfo[row].rightLine); //限幅

        rowInfo[row].width = rowInfo[row].rightLine - rowInfo[row].leftLine;
        rowInfo[row].midLine = (rowInfo[row].rightLine + rowInfo[row].leftLine) / 2;


        if (rowInfo[row].width <= 7) //重新确定可视距离
        {
            imgInfo.top = row + 1;
            break;
        }
        else if (rowInfo[row].rightLine <= 10 || rowInfo[row].leftLine >= WIDTH - 10)
        {
            imgInfo.top = row + 1;
            break;
        } //当图像宽度小于0或者左右边达到一定的限制时，则终止巡边

#ifdef DEBUG
    // ("rowInfo[%3d].leftLine = %4d      ", row, rowInfo[row].leftLine);
    // printf("rowInfo[%3d].leftLine = %4d      Status = %3d\n", row, rowInfo[row].leftLine, rowInfo[row].leftStatus);
#endif
    }
    return;
}
#undef LIMIT_L
#undef LIMIT_R
#undef LIMIT_H

void basic_getJumpPointFromDet(uint8_t *row, int L,int R, EdgePointTypedef *Q, LineTypeEnum type) //第一个参数是要查找的数组
{
    if (type == LEFT) //扫描左边线
    {
        for (int i = R; i >= L; --i)
        {
            if (*(row + i)&& !(*(row + i - 1))) //由白变黑
            {
                Q->posX = i - 1;  //记录左边线
                Q->type = EXIST; /////正确跳变
                break;
            }
            else if (i == (L + 1)) //若果扫到最后也没找到
            {
                if (*(row + (L + R) / 2)) //如果中间是白的
                {
                    Q->posX = (L + R) / 2; //认为左边线是中点
                    Q->type = LOST;          /////非正确跳变且中间为白，认为没有边
                    break;
                }
                else ////非正确跳变且中间为黑
                {
                    Q->posX = R;  //如果中间是黑的
                    Q->type = JUMP; //左边线直接最大值，认为是大跳变
                    break;
                }
            }
        }
    }
    else if (type == RIGHT)
    {
        for (int i = L; i <= R; ++i)
        {
            if (*(row + i) && !(*(row + i + 1))) //由黑变白
            {
                Q->posX = i + 1;  //记录右边线
                Q->type = EXIST; /////正确跳变
                break;
            }
            else if (i == (R - 1)) //若果扫到最后也没找到
            {
                if (*(row + (L + R) / 2)) //如果中间是白的
                {
                    Q->posX = (L + R) / 2; //认为左边线是中点
                    Q->type = LOST;        //非正确跳变且中间为白，认为没有边
                    break;
                }
                else ////非正确跳变且中间为黑
                {
                    Q->posX = L;  //如果中间是黑的
                    Q->type = JUMP; //右边线直接最大值，认为是大跳变
                    break;
                }
            }
        }
    }
}

void advance_repairLine(void) /////绘制延长线并重新确定中线
{
    int FTSite = 0,TFSite = 0;
    float kL = 0.0, kR = 0.0;
    if (needExternL != 'F')
    {
        for (int row = HEIGHT-6; row >= (imgInfo.top + 4); row--)
        //从第五行开始网上扫扫到顶边下面两行   多段补线
                        //不仅仅只有一段
        {
            if (rowInfo[row].leftStatus == LOST) //如果本行左边界没扫到但扫到的是白色，说明本行没有左边界点
            {
                if (rowInfo[row + 1].leftLine >= WIDTH - 10) //如果左边界实在是太右边
                {
                    imgInfo.top = row + 1;
                    break; //直接跳出（极端情况）
                }

                while (row >= (imgInfo.top + 4)) //此时还没扫到顶边
                {
                    row--; //继续往上扫
                    if (rowInfo[row].leftStatus == EXIST &&
                        rowInfo[row - 1].leftStatus == EXIST &&
                        rowInfo[row - 2].leftStatus == EXIST &&
                        rowInfo[row - 2].leftLine > 0 &&
                        rowInfo[row - 2].leftLine < WIDTH - 10) //如果扫到本行出现了并且本行以上连续三行都有左边界点（左边界在空白上方）
                    {
                        FTSite = row - 2; //把本行上面的第二行存入FTsite
                        break;
                    }
                }

                kL = ((float)(rowInfo[FTSite].leftLine - rowInfo[TFSite].leftLine)) / ((float)(FTSite - TFSite)); //左边界的斜率：列的坐标差/行的坐标差
                if (FTSite > imgInfo.top)
                {
                    for (int y = TFSite; y >= FTSite; y--)
                         //从第一次扫到的左边界的下面第二行的坐标开始往上扫直到空白上方的左边界的行坐标值
                    {
                        rowInfo[y].leftLine = (int)(kL * ((float)(y - TFSite))) + rowInfo[TFSite].leftLine;
                        //将这期间的空白处补线（补斜线），目的是方便图像处理
                    }
                }
            }
            else
                TFSite = row + 2; //如果扫到了本行的左边界，该行存在这里面，（算斜率）
        }
    }

    if (needExternR != 'F')
    {
        for (int row = HEIGHT-6; row >= (imgInfo.top + 4); row--) //从第五行开始网上扫扫到顶边下面两行
        {
            if (rowInfo[row].rightStatus == LOST) //如果本行右边界没扫到但扫到的是白色，说明本行没有右边界点，但是处于赛道内的
            {
                if (rowInfo[row + 1].rightLine <= 10) //如果右边界实在是太左边
                {
                    imgInfo.top = row + 1; //直接跳出
                    break;
                }
                while (row >= (imgInfo.top + 4)) //此时还没扫到顶边下面两行
                {
                    row--;
                    if (rowInfo[row].rightStatus == EXIST &&
                        rowInfo[row - 1].rightStatus == EXIST &&
                        rowInfo[row - 2].rightStatus == EXIST &&
                        rowInfo[row - 2].rightLine < WIDTH-1 &&
                        rowInfo[row - 2].rightLine > 10)
                        //如果扫到本行出现了并且本行以上连续三行都有左边界点（左边界在空白上方）
                    {
                        FTSite = row - 2; // 把本行上面的第二行存入FTsite
                        break;
                    }
                }

                kR = ((float)(rowInfo[FTSite].rightLine - rowInfo[TFSite].rightLine)) / ((float)(FTSite - TFSite));
                    //右边界的斜率：列的坐标差/行的坐标差
                if (FTSite > imgInfo.top)
                {
                    for (int y = TFSite; y >= FTSite; y--) //从第一次扫到的右边界的下面第二行的坐标开始往上扫直到空白上方的右边界的行坐标值
                    {
                        rowInfo[y].rightLine = (int)(kR * ((float)(y - TFSite))) + rowInfo[TFSite].rightLine;
                            //将这期间的空白处补线（补斜线），目的是方便图像处理
                    }
                }
            }
            else
                TFSite = row + 2; //如果本行的右边界找到了，则把该行下面第二行坐标送个TFsite
        }
    }
    for (int row = HEIGHT-1; row >= imgInfo.top; row--)
    {
        rowInfo[row].midLine = (rowInfo[row].leftLine + rowInfo[row].rightLine) / 2; //扫描结束，把这一块经优化之后的中间值存入
        rowInfo[row].width = rowInfo[row].rightLine-rowInfo[row].leftLine; //把优化之后的宽度存入
    }
}

//出现丢边的时候  重新确定无边行的中线
void advance_midLineFilter(void)
{
    float kR = 0.0;
    int y;
    for (int row = HEIGHT - 2; row >= imgInfo.top + 5; --row) //从开始位到停止位
    {
        if( row <= HEIGHT - 15 &&
            rowInfo[row].leftStatus == LOST &&
            rowInfo[row].rightStatus == LOST &&
            rowInfo[row - 1].leftStatus == LOST &&
            rowInfo[row - 1].rightStatus == LOST) //当前行左右都无边，而且在前105行   滤波
        {
            y = row;
            while (y >= (imgInfo.top + 5))
            {
                y--;
                if (rowInfo[y].leftStatus == 'T' && rowInfo[y].rightStatus == 'T') //寻找两边都正常的，找到离本行最近的就不找了
                {
                    kR = (float)(rowInfo[y - 1].midLine - rowInfo[row + 2].midLine) / (float)(y - 1 - row - 2); //算斜率
                    int CenterTemp = rowInfo[row + 2].midLine;
                    int LineTemp = row + 2;
                    while (row >= y)
                    {
                        rowInfo[row].midLine = (int)(CenterTemp + kR * (float)(row - LineTemp)); //用斜率补
                        row--;
                    }
                    break;
                }
            }
        }
        rowInfo[row].midLine = (rowInfo[row - 1].midLine + 2 * rowInfo[row].midLine) / 3; //求平均，应该会比较滑  本来是上下两点平均
    }
}

/**
 * @description: 根据选择范围的数据对应用范围补线,支持左中右线
 * @param {uint8_t} select_top
 * @param {uint8_t} select_bottom
 * @param {uint8_t} apply_top
 * @param {uint8_t} apply_bottom
 * @param {RoadTypeEnum} type
 * @return {*}
 */
void custom_repairLine(uint8_t select_top, uint8_t select_bottom, uint8_t apply_top, uint8_t apply_bottom, LineTypeEnum type)
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
    if (
        imgInfo.RoadType != Cross      //赛道类型清0  if里面的情况都需要积分退出 不能在这儿请0
        && imgInfo.RoadType != Circle_L
        &&imgInfo.RoadType != Circle_R
        &&imgInfo.RoadType != Slope
      )

    imgInfo.RoadType = Road_None;

    if (
        imgInfo.RoadType != Cross
        && imgInfo.RoadType != Circle_L
        &&imgInfo.RoadType != Circle_R
        &&imgInfo.RoadType != Slope
      )
    straight_detect();

    straight_speedUpDetect();

    cross_detect();

    if(imgInfo.RoadType != Cross)
    {
        circle_detect();
    }

    if(imgInfo.RoadType != Cross)
    {
        fork_detect();
    }

    circle_repairLine();
    fork_repairLine();
    return ;
}

void straight_detect(void)
{
    float midK = 0, sum = 0, variance = 0;//中线斜率, 差方和, 中线与拟合方差
    midK = (rowInfo[HEIGHT - 5].midLine - rowInfo[imgInfo.top + 1].midLine) /
             (float)(HEIGHT - 5 - imgInfo.top - 1);
    float mid_tmp = 0;
    for (int row = HEIGHT - 5; row > imgInfo.top + 1; row--)
    {
        mid_tmp = rowInfo[HEIGHT - 5].midLine - midK * (HEIGHT - 5 - row) + 0.5;
        sum += (rowInfo[row].midLine - mid_tmp) * (rowInfo[row].midLine - mid_tmp);
    }

    variance = sum / (HEIGHT - 5 - imgInfo.top - 1);
    if (variance < ConstData.kImageStraightLineVarianceTh && imgInfo.top <= 25)
    {
        imgInfo.RoadType = Straight;
    }
}
void straight_speedUpDetect(void)
{
    int sum = 0;
    float variance = 0;  //中线与中间方差
    for (int row = HEIGHT - 5; row > imgInfo.top + 1; row--)
    {
        sum += (rowInfo[row].midLine - (WIDTH/2)) * (rowInfo[row].midLine - (WIDTH/2));
    }
    variance = (float)sum / (HEIGHT - 5 - imgInfo.top - 1);

    if (variance < ConstData.kImageStraightLineSpeedUpVarianceTh && imgInfo.top <= 22 && imgInfo.RoadType != Slope)
    {
        imgInfo.straight_needSpeedUP = 1;
    }
    else
        imgInfo.straight_needSpeedUP = 0;
}


/**
 * @description: 弯道特殊扫线
 * @param {*}
 * @return {*}
 */
void turn_detect(void)
{
    for(int i = max(leftDownStart,rightDownStart); i > imgInfo.top; --i)
    {
        if(rowInfo[i].leftStatus == EXIST)
        {
            if(rowInfo[i].rightStatus == EXIST)
                rowInfo[i].midLine = (rowInfo[i].leftLine + rowInfo[i].rightLine) / 2;
            else
                rowInfo[i].midLine = rowInfo[i].leftLine + regWidth[i] / 2;
        }
        else
        {
            if(rowInfo[i].rightStatus == EXIST)
                rowInfo[i].midLine = rowInfo[i].rightLine - regWidth[i] / 2;
            else
                rowInfo[i].midLine = rowInfo[i+1].midLine + rowInfo[i+1].midLine - rowInfo[i+2].midLine;
        }
    }
    return ;
}

/**
 * @description: 十字路口扫线
 * @param {*}
 * @return {*}
 */
void cross_detect(void)
{


    // //左线
    // float k = (float)(rowInfo[leftUpJump].leftLine - rowInfo[leftDownJump].leftLine) / (leftUpJump - leftDownJump);
    // float b = (float)rowInfo[leftDownJump].leftLine - k * leftDownJump;
    // // leftLine[leftDownJump] = k * leftDownJump + b;

    // add_line(k, b, leftUpJump, leftDownJump, LEFT);

    // //右线
    // k = (float)(rowInfo[rightUpJump].rightLine - rowInfo[rightDownJump].rightLine) / (rightUpJump - rightDownJump);
    // b = (float)rowInfo[rightDownJump].rightLine - k * rightDownJump;

    // add_line(k, b, rightUpJump, rightDownJump, RIGHT);

    // for(int i = min(leftUpJump, rightUpJump); i <= max(leftDownJump, rightDownJump); ++i)
    // {
    //     rowInfo[i].midLine = (rowInfo[i].leftLine + rowInfo[i].rightLine) / 2;
    // }
    // custom_repairLine(leftUpJump,leftDownStart,leftDownStart,imgInfo.bottom, LEFT);
    // custom_repairLine(rightUpJump,rightDownStart,rightDownStart,imgInfo.bottom, RIGHT);
    // for(int i = min(rightDownStart, leftDownStart); i <= imgInfo.bottom; ++i)
    // {
    //     rowInfo[i].midLine = (rowInfo[i].leftLine + rowInfo[i].rightLine ) / 2;
    // }
    // return ;
}
void cross_detect_withLeft(void)
{
    float leftK, leftB;
    leftK = (float)(rowInfo[leftUpJump].leftLine - rowInfo[leftDownJump].leftLine) / (leftUpJump - leftDownJump);
    leftB = (float)rowInfo[leftUpJump].leftLine - leftK * leftUpJump;
    // least_squares(&rightK, &rightB, max(imgInfo.top,rightUpJump - 4), rightUpJump, RIGHT);
    add_line(leftK, leftB, leftUpJump, leftDownJump, LEFT);

    for(int i = imgInfo.bottom; i >= imgInfo.top; --i)
    {
        if(rowInfo[i].rightStatus != EXIST)
        {
            rowInfo[i].midLine = rowInfo[i].leftLine + regWidth[i] / 2;
        }
        else if(rowInfo[i].leftStatus != EXIST)
        {
            rowInfo[i].midLine = rowInfo[i].rightLine - regWidth[i] / 2;
        }
    }
    basic_repairLine();
    return ;
}
void cross_detect_withRight(void)
{
    float rightK, rightB;
    rightK = (float)(rowInfo[rightUpJump].rightLine - rowInfo[rightDownJump].rightLine) / (rightUpJump - rightDownJump);
    rightB = (float)rowInfo[rightUpJump].rightLine - rightK * rightUpJump;
    // least_squares(&leftK, &leftB, max(imgInfo.top,leftUpJump - 4), leftUpJump, LEFT);
    add_line(rightK, rightB, rightUpJump, rightDownJump, RIGHT);
    
    for(int i = imgInfo.bottom; i >= imgInfo.top; --i)
    {
        if(rowInfo[i].leftStatus != EXIST)
        {
            rowInfo[i].midLine = rowInfo[i].rightLine - regWidth[i] / 2;
        }
        else if(rowInfo[i].rightStatus != EXIST)
        {
            rowInfo[i].midLine = rowInfo[i].leftLine + regWidth[i] / 2;
        }
    }
    basic_repairLine();
    return ;
}

void fork_detect()
{
    uint8_t * PicTemp = NULL;

    // 第一特征 判断角度
    for (int row = HEIGHT - 10; row > (imgInfo.top + 6); row--) //防止row溢出
    {
        if ((rowInfo[row].rightStatus == EXIST && rowInfo[row + 1].rightStatus == EXIST) ||
            (rowInfo[row].leftStatus == EXIST && rowInfo[row + 1].leftStatus == EXIST)) //进三叉的时候一般会看见左右两边120*的圆角
        {
            if  ((

                    (rowInfo[row].leftLine - rowInfo[row - 6].leftLine) > 2 &&
                    (rowInfo[row].leftLine - rowInfo[row - 6].leftLine) < 8 &&
                    (rowInfo[row].leftLine - rowInfo[row + 6].leftLine) > 2 &&
                    (rowInfo[row].leftLine - rowInfo[row + 6].leftLine) < 8)
                    || (//左边的角
                    (rowInfo[row - 6].rightLine - rowInfo[row].rightLine) > 3 &&
                    (rowInfo[row - 6].rightLine - rowInfo[row].rightLine) < 8 &&
                    (rowInfo[row + 6].rightLine - rowInfo[row].rightLine) > 2 &&//右边的角 看到一个就可以  因为看到两个经常漏判
                    (rowInfo[row + 6].rightLine - rowInfo[row].rightLine) < 8

                ))//阈值需要调整
            {
                fork_flag_1 = 'T'; //表示近端的角点特征找到
                fork_spoint_1_Y = row; //记录第一特征点的行数
                break;
            }

            else
            {
                fork_flag_1 = 'F'; //没找到
            }
        }
    }

    int wide = 0;
    //第二特征 找黑色三角块  并运算得到相关图像信息
    for (int row = fork_spoint_1_Y; row > imgInfo.top; row--) //  从第一特征点开始往上搜索
    {
        PicTemp = imageBin[row];
        for (int x = rowInfo[row].leftLine; x < WIDTH - 10; x++) //找三叉口黑色三角块
        {
            if ((*(PicTemp + x) != 0) && (*(PicTemp + x + 1) == 0) && (*(PicTemp + x + 2) == 0))
            //找到黑色角快的左边
            {
                rowInfo[row].fork_L = x + 1; //记录此时的左黑边界
                break;
            }
            else
                rowInfo[row].fork_L = MISS; //没找到就在中点
        }

        for (int x = rowInfo[row].rightLine; x > WIDTH / 2; x--) //找三叉口黑色三角块
        {
            if ((*(PicTemp + x) == 0) && (*(PicTemp + x - 1) == 0) && (*(PicTemp + x + 1) != 0))
            //找到黑色角快的右边
            {
                rowInfo[row].fork_R = x; //记录此时的右黑边界
                break;
            }
            else
                rowInfo[row].fork_R = MISS; //没找到就在中点
        }

        for (int x = rowInfo[row].fork_L; x <= rowInfo[row].fork_R; x++)
        {
            if (rowInfo[row].fork_L == MISS ||
                rowInfo[row].fork_R == MISS) //如果是中点值那么没找到
                break;
            else if ((*(PicTemp + x) == 0)) //数数左黑和右黑之间的黑点数
            {
                wide++; //计算找到三角块的本行黑点数
            }
        }
        rowInfo[row].fork_blackWidth = wide; //记录这个宽度
        rowInfo[row].fork_black_k = rowInfo[row].fork_blackWidth / rowInfo[row].width; //图像黑点比例
        wide = 0;                                               //清0
    }

    //判断是否为三叉的黑色三角块
    for (int row = fork_spoint_1_Y; row >= (imgInfo.top + 1); row--)
    {
        if ((rowInfo[row].fork_blackWidth - rowInfo[row + 3].fork_blackWidth) >= 2 &&
            //如果这个左黑和右黑之间黑点数比较多 并且满足三角形的形状
            //再和斜十字区分
            rowInfo[row].fork_blackWidth > 40 &&
            rowInfo[row + 1].fork_blackWidth > 30 &&
            rowInfo[row - 1].fork_blackWidth > 30 &&
            rowInfo[row].fork_L < WIDTH / 2 && //滤除斜十字
            rowInfo[row].fork_R > WIDTH / 2)
        {

            ForkLinePointx_r = rowInfo[row].fork_R; //用于补线的点
            ForkLinePointx_l = rowInfo[row].fork_L;
            ForkLinePointy = row;

            fork_spoint_2_Y = row;           //记录第二特征点的行数
            if (fork_spoint_1_Y - fork_spoint_2_Y > 10) // g
            {
                fork_flag_2 = 'T'; //当两特征点行数大于10   才判断为入环特征点  用于防误判
                break;
            }
        }
        else
            fork_flag_2 = 'F';
    }

    // 综合上述信息判断
    if ((fork_flag_1 == 'T' && fork_flag_2 == 'T') || (fork_flag_tot == 'T' && fork_flag_2 == 'T'))
        fork_flag_tot = 'T';
    else
        fork_flag_tot = 'F';

    if (fork_flag_tot == 'T')
        imgInfo.RoadType = Fork_In;

#ifdef DEBUG
    // printf("fork_spoint_1_Y = %d\n", (int)fork_spoint_1_Y);
    // printf("fork_flag_1 = %c\n", fork_flag_1);
    // printf("fork_flag_2 = %c\n", fork_flag_2);
    // printf("fork_flag_tot = %c\n", fork_flag_tot);
#endif

}

void fork_repairLine()
{
    float Det_Fork_L;
    // float Det_Fork_R;

    if (imgInfo.RoadType == Fork_In )//|| imgInfo.RoadType == Fork_Out ) // 右补线
    {
        Det_Fork_L = 1.05 * ForkLinePointx_r / (HEIGHT - 5 - ForkLinePointy);
        for (int row = HEIGHT - 5; row > imgInfo.top; row--)
        {
            int temp = (int)(rowInfo[HEIGHT - 5].leftLine + Det_Fork_L * (HEIGHT - 5  - row));
            if (temp >= WIDTH)
            {
                temp = WIDTH - 1;
            }
            if (temp > rowInfo[row].leftLine)
            {
                rowInfo[row].leftLine = temp;
                if (imageBin[row][rowInfo[row].leftLine] == 0)
                {
                    for (int x = rowInfo[row].leftLine; x < rowInfo[row].rightLine; x++)
                    {
                        if (imageBin[row][x] == 1 && imageBin[row][x + 1] == 1)
                        {
                            rowInfo[row].leftLine = x;
                        }
                    }
                }
            }
            rowInfo[row].midLine = (rowInfo[row].rightLine + rowInfo[row].leftLine) / 2;
            rowInfo[row].width = rowInfo[row].rightLine - rowInfo[row].leftLine; //宽度更新
        }
    }
}
int color_TogglePos_left[10] = {0}, color_TogglePos_right[10] = {0};
int color_toggleCnt_left = 0, color_toggleCnt_right = 0;
// 黑->白->黑->白->黑 四个跳变点(防止可能的越界)
uint8_t isCircle_flag_1 = 'F',isCircle_flag_2 = 'F',isCircle_flag_3 = 'F';
void circle_judge_1(void)
{
    color_toggleCnt_left = 0, color_toggleCnt_right = 0; // tmp
    isCircle_flag_1 = 'F'; // tmp

    for(int row = HEIGHT - 5; row > imgInfo.top + 2; row--)
    {
        // 左线交错变色检测
        if (!(color_toggleCnt_left % 2) && !imageBin[row][rowInfo[row].leftLine] && !imageBin[row - 1][rowInfo[row - 1].leftLine])
        {
            continue ;
        }
        if (color_toggleCnt_left % 2 && imageBin[row][rowInfo[row].leftLine] && imageBin[row - 1][rowInfo[row - 1].leftLine])
        {
            continue;
        }
        if (!(color_toggleCnt_left % 2) && !imageBin[row][rowInfo[row].leftLine] && imageBin[row - 1][rowInfo[row - 1].leftLine])
            color_TogglePos_left[color_toggleCnt_left++] = row;
        else if (color_toggleCnt_left % 2 && imageBin[row][rowInfo[row].leftLine] && !imageBin[row - 1][rowInfo[row - 1].leftLine])
            color_TogglePos_left[color_toggleCnt_left++] = row - 1;

        //右线交错变色检测
        if (!(color_toggleCnt_right % 2) && !imageBin[row][rowInfo[row].rightLine] && !imageBin[row - 1][rowInfo[row - 1].rightLine])
        {
            continue ;
        }
        if (color_toggleCnt_right % 2 && imageBin[row][rowInfo[row].rightLine] && imageBin[row - 1][rowInfo[row - 1].rightLine])
        {
            continue;
        }
        if (!(color_toggleCnt_right % 2) && !imageBin[row][rowInfo[row].rightLine] && imageBin[row - 1][rowInfo[row - 1].rightLine])
            color_TogglePos_left[color_toggleCnt_right++] = row;
        else if (color_toggleCnt_right % 2 && imageBin[row][rowInfo[row].rightLine] && !imageBin[row - 1][rowInfo[row - 1].rightLine])
            color_TogglePos_left[color_toggleCnt_right++] = row;
    }

    if ((color_toggleCnt_left >= 3 && color_toggleCnt_right <= 1) ||
        (color_toggleCnt_left <= 1 && color_toggleCnt_right >= 3))
        isCircle_flag_1 = 'T';
    else
        isCircle_flag_1 = 'F';
}
PixelTypedef PointSerial[HEIGHT<<1]; // TODO 这里移到函数内
int16_t SerialCnt = 0;//种子生长法计数
void circle_judge_2(void)
{
    isCircle_flag_2 = 'F';

    SerialCnt = 0;
    // int16_t SerialCnt = 0;//种子生长法计数


    PixelTypedef start;

    const int dx[8] = {1, 1, 0,-1,-1,-1, 0, 1,};
    const int dy[8] = {0,-1,-1,-1, 0, 1, 1, 1,};
    /**
     * 定义 # 周围为
     *
     *   3 2 1
     *   4 # 0
     *   5 6 7
     *
     */
    int16_t x, y, tx, ty, s, SearchCompleteFlag;
    if (color_toggleCnt_left >= 3 && color_toggleCnt_right <= 1) // 意味着或许是左环岛
    {
        //记录起始点
        start.y = color_TogglePos_left[1];
        start.x = rowInfo[start.y].leftLine;
        PointSerial[SerialCnt++] = start;
        //开始八邻域扫线
        s = 5;
        x = start.x;
        y = start.y;
        SearchCompleteFlag = 0; // 搜索完成标志位

        while(1)
        {
            //强制结束,防止扫过头
            if(y <= min(imgInfo.top, color_TogglePos_left[3] - 3) || SerialCnt > (HEIGHT<<1)) // 小于扫线终止或者下一个突变减去阈值
            {
                isCircle_flag_2 = 'F';
                break;
            }
            // printf(" $%d %d$ \n", x, y);
            //逆时针旋转扫线
            for( ; ; ++s)
            {
                s = (s + 8) % 8;
                // if(s == 8) s = 0;

                tx = x + dx[s];// 周围元素的坐标
                ty = y + dy[s];
                if(tx == start.x && ty == start.y)// 扫到起始点，搜索完成
                {
                    SearchCompleteFlag = 1;
                    break;
                }
                if(tx == 0 || tx == 1) continue; //最左两列强制看作白
                if(!imageBin[ty][tx])// 黑色（跳变）更新x和y
                {
                    x = tx;
                    y = ty;
                    s =  (s - 2 + 8) % 8; // 回退一个直角
                    break;
                }
            }
            if(SearchCompleteFlag == 1)// 搜索完成退出
            {
                isCircle_flag_2 = 'T';
                break;
            }

            ++SerialCnt;// 记步数
            PointSerial[SerialCnt].x = tx;
            PointSerial[SerialCnt].y = ty;
        }
    }
    else if (color_toggleCnt_left <= 1 && color_toggleCnt_right >= 3) // 意味着或许是右环岛
    {
        //记录起始点
        start.y = color_TogglePos_right[1];
        start.x = rowInfo[start.y].leftLine;
        //开始八邻域扫线
        s = 7;
        x = start.x;
        y = start.y;
        SearchCompleteFlag = 0; // 搜索完成标志位

        while(1)
        {
            //强制结束,防止扫过头
            if(y <= min(imgInfo.top, color_TogglePos_right[3] - 3) || SerialCnt > (HEIGHT<<1)) // 小于扫线终止或者下一个突变减去阈值
            {
                isCircle_flag_2 = 'F';
                break;
            }
            // printf(" $%d %d$ \n", x, y);
            //逆时针旋转扫线
            for( ; ; ++s)
            {
                s = (s + 8) % 8;
                // if(s == 8) s = 0;

                tx = x + dx[s];// 周围元素的坐标
                ty = y + dy[s];
                if(tx == start.x && ty == start.y)// 扫到起始点，搜索完成
                {
                    SearchCompleteFlag = 1;
                    break;
                }
                if(tx == WIDTH - 1 || tx == WIDTH - 2) continue; //最左两列强制看作白
                if(!imageBin[ty][tx])// 黑色（跳变）更新x和y
                {
                    x = tx;
                    y = ty;
                    s =  (s - 2 + 8) % 8; // 回退一个直角
                    break;
                }
            }
            if(SearchCompleteFlag == 1)// 搜索完成退出
            {
                isCircle_flag_2 = 'T';
                break;
            }

            ++SerialCnt;// 记步数
        }
    }



}
void circle_judge_3(void)
{

}

int up_point = MISS;
const int circle_rad = 8;
const int circle_getPoint_del = 5;
float circle_k = 1.02;

void circle_detect(void)
{
    circle_judge_1();
    circle_judge_2();
    if (isCircle_flag_1 == 'T' && isCircle_flag_2 == 'T' && imgInfo.CircleStatus == CIRCLE_NOT_FIND)
    {
#ifdef DEBUG
        printf("OK CIRCLE\n");
#endif
        imgInfo.CircleStatus = CIRCLE_FIND; // 标记 发现环岛 状态
    }

    if (imgInfo.CircleStatus == CIRCLE_FIND)
    {
        if (color_toggleCnt_left >= 3 && color_toggleCnt_right <= 1 && imgInfo.RoadType != Circle_R)
        {
            imgInfo.RoadType = Circle_L;
        }
        else if (color_toggleCnt_left <= 1 && color_toggleCnt_right >= 3 && imgInfo.RoadType != Circle_L)
        {
            imgInfo.RoadType = Circle_R;
        }
        else
        {
            imgInfo.CircleStatus = CIRCLE_IN;
        }
    }

    if(imgInfo.CircleStatus == CIRCLE_IN)
    {

        if(imgInfo.RoadType  == Circle_L)
        {

            for (int row = HEIGHT - 10; row > (imgInfo.top + 10); row--) //防止row溢出
            {
                if (rowInfo[row].rightStatus == EXIST && rowInfo[row + 1].rightStatus == EXIST) //进三叉的时候一般会看见左右两边120*的圆角
                {
                    if (
                            (rowInfo[row - 4].rightLine - rowInfo[row].rightLine) > 2 &&
                            (rowInfo[row - 4].rightLine - rowInfo[row].rightLine) < 15 &&
                            (rowInfo[row + 6].rightLine - rowInfo[row].rightLine) > 2 &&
                            (rowInfo[row + 6].rightLine - rowInfo[row].rightLine) < 15
                    )//阈值需要调整
                    {
                        imgInfo.CircleStatus = CIRCLE_OUT;
                        break;
                    }

                }
            }
        }
        else if (imgInfo.RoadType  == Circle_R)
        {
            for (int row = HEIGHT - 10; row > (imgInfo.top + 10); row--) //防止row溢出
            {
                if (rowInfo[row].leftStatus == EXIST && rowInfo[row + 1].leftStatus == EXIST) //进三叉的时候一般会看见左右两边120*的圆角
                {
                    if (
                            (rowInfo[row].leftLine - rowInfo[row - 4].leftLine) > 2 &&
                            (rowInfo[row].leftLine - rowInfo[row - 4].leftLine) < 15 &&
                            (rowInfo[row].leftLine - rowInfo[row + 6].leftLine) > 2 &&
                            (rowInfo[row].leftLine - rowInfo[row + 6].leftLine) < 15
                    )//阈值需要调整
                    {
                        imgInfo.CircleStatus = CIRCLE_OUT;
                        break;
                    }

                }
            }
        }
    }

    if(imgInfo.CircleStatus == CIRCLE_OUT)
    {

        if(imgInfo.RoadType == Circle_L)
        {

            int variance = 0;

            float pred_k = (rowInfo[HEIGHT - 5].rightLine - rowInfo[imgInfo.top + 5].rightLine)
                            / (HEIGHT - 5 - (imgInfo.top + 5));
            float pred_b = rowInfo[(HEIGHT + imgInfo.top) / 2].rightLine - pred_k * ((HEIGHT + imgInfo.top) / 2);

            for (int row = HEIGHT - 5; row > (imgInfo.top + 5); row--) //防止row溢出
            {
                printf("rowInfo[%3d].rightLine = %3d    pred_rightLine = %3d \n", row, (int)rowInfo[row].rightLine, (int)(pred_k * row + pred_b));
                variance = variance + (rowInfo[row].rightLine - (pred_k * row + pred_b)) * (rowInfo[row].rightLine - (pred_k * row + pred_b));
            }
            variance = variance / (HEIGHT - 5 - (imgInfo.top + 5));
            #ifdef DEBUG
                printf("pred_k: %f\n", pred_k);
                printf("pred_b: %f\n", pred_b);
                printf("variance = %d\n", variance);
            #endif
            if (variance < ConstData.kImageCircleOutVarianceTh)
            {
                imgInfo.CircleStatus = CIRCLE_OFF;
            }
        }
        else if (imgInfo.RoadType == Circle_R)
        {
            int variance = 0;

            float pred_k = (rowInfo[HEIGHT - 5].leftLine - rowInfo[imgInfo.top + 5].leftLine)
                            / (HEIGHT - 5 - (imgInfo.top + 5));
            float pred_b = rowInfo[(HEIGHT + imgInfo.top) / 2].leftLine - pred_k * ((HEIGHT + imgInfo.top) / 2);

            for (int row = HEIGHT - 5; row > (imgInfo.top + 5); row--) //防止row溢出
            {
                variance = variance + (rowInfo[row].leftLine - (pred_k * row + pred_b)) * (rowInfo[row].leftLine - (pred_k * row + pred_b));
            }

            if (variance <= ConstData.kImageCircleOutVarianceTh)
            {
                imgInfo.CircleStatus = CIRCLE_OFF;
            }
        }

    }

    if(imgInfo.CircleStatus == CIRCLE_OFF)
    {

        imgInfo.CircleStatus = CIRCLE_NOT_FIND;
        imgInfo.RoadType = Road_None;
    }

}

void circle_repairLine(void)
{
    //入环补线
    if (imgInfo.CircleStatus == CIRCLE_FIND) //此时处于入环状态
    {
        //左圆环
        if (imgInfo.RoadType == Circle_L)
        {
            for (int y = HEIGHT - 5; y > imgInfo.top + 2; y--) //找到补线点
            {
                if (rowInfo[y].leftLine + circle_getPoint_del < rowInfo[y - 2].leftLine)
                {
                    up_point = y - 2;
                    imgInfo.top = up_point;
                    break;
                }
            }

            if (up_point != MISS)
            {
                for (int y = up_point; y < HEIGHT - 5; y++)
                {
                    int temp = rowInfo[up_point].leftLine + circle_rad * sqrt(y - up_point);//根号曲线拟合
                    if (temp < rowInfo[y].rightLine)
                    {
                        rowInfo[y].rightLine = temp;
                        if (imageBin[y][rowInfo[y].rightLine] == 0)
                        {
                            for (int x = rowInfo[y].rightLine; x > rowInfo[y].leftLine; x--)
                            {
                                if (imageBin[y][x] && imageBin[y][x - 1])
                                {
                                    rowInfo[y].rightLine = x;
                                }
                            }
                        }
                    }
                    rowInfo[y].midLine = (rowInfo[y].leftLine + rowInfo[y].rightLine) / 2;
                }
            }
        }
        //右圆环
        if (imgInfo.RoadType == Circle_R)
        {
            for (int y = HEIGHT - 5; y > imgInfo.top + 2; y--) //找到补线点
            {
                if (rowInfo[y].rightLine - circle_getPoint_del > rowInfo[y - 2].rightLine)
                {
                    up_point = y - 2;
                    imgInfo.top = up_point;
                    break;
                }
            }

            if (up_point != MISS)
            {
                for (int y = up_point; y < HEIGHT - 5; y++)
                {
                    int temp = rowInfo[up_point].rightLine - circle_rad * sqrt(y - up_point);//根号曲线拟合
                    if (temp > rowInfo[y].leftLine)
                    {
                        rowInfo[y].leftLine = temp;
                        if (imageBin[y][rowInfo[y].leftLine] == 0)
                        {
                            for (int x = rowInfo[y].leftLine; x < rowInfo[y].rightLine; x++)
                            {
                                if (imageBin[y][x] && imageBin[y][x + 1])
                                {
                                    rowInfo[y].leftLine = x;
                                }
                            }
                        }
                    }

                    rowInfo[y].midLine = (rowInfo[y].leftLine + rowInfo[y].rightLine) / 2;
                }
            }
        }
    }

    //出环补线
    if (imgInfo.CircleStatus == CIRCLE_OUT) //此时处于出环状态
    {
        //左圆环
        float Det_R;
        if (imgInfo.RoadType == Circle_L)
        {
            Det_R = circle_k * (WIDTH - 1) / (HEIGHT - 5 - imgInfo.top);
            for (int y = HEIGHT -5; y > imgInfo.top; y--)
            {
                int temp = (int)(rowInfo[HEIGHT - 5].rightLine - Det_R * (HEIGHT - 5 - y));
                if (temp < 0)
                {
                    temp = 0;
                }
                if (temp < rowInfo[y].rightLine)
                {
                    rowInfo[y].rightLine = temp;
                    rowInfo[y].leftLine = 0;
                    rowInfo[y].midLine = (rowInfo[y].leftLine + rowInfo[y].rightLine) / 2;
                    rowInfo[y].width = rowInfo[y].rightLine - rowInfo[y].leftLine;
                }
            }
        }

        float Det_L;

        if (imgInfo.RoadType == Circle_R)
        {
            Det_L = circle_k * (WIDTH - 1) / (HEIGHT - 5 - imgInfo.top);
            for (int y = HEIGHT - 5; y > imgInfo.top; y--)
            {
                int temp = (int)(rowInfo[HEIGHT - 5].leftLine + Det_L * (HEIGHT - 5 - y));
                if (temp > WIDTH - 1)
                {
                    temp = WIDTH - 1;
                }
                if (temp > rowInfo[y].leftLine)
                {
                    rowInfo[y].leftLine = temp;
                    rowInfo[y].rightLine = WIDTH - 1;
                    rowInfo[y].midLine = (rowInfo[y].leftLine + rowInfo[y].rightLine) / 2;
                    rowInfo[y].width = rowInfo[y].rightLine - rowInfo[y].leftLine;
                }
            }
        }
    }
}

/**
 * @description: 获取当前位置和中线位置的偏差，传递给PID处理
 * @param {*}
 * @return {*}
 */
void get_error(void)//TODO 实现动态调整前瞻
{
    int avePos = 0, predictedPass = 0;
    float spd = 2.5;// = get_speed() 之后写成由speed动态选择前瞻的模式
    predictedPass = max (imgInfo.bottom - (int)(spd * 10), imgInfo.top + 3); //之后根据实际情况写出关系式
    avePos = (rowInfo[predictedPass].midLine + rowInfo[predictedPass + 1].midLine + rowInfo[predictedPass + 2].midLine) / 3;

    imgInfo.error = avePos-(WIDTH/2);
    // printf("%d :   %d\n", predictedPass, rowInfo[predictedPass].midLine);
    // printf("avePos = %d\nimgInfo.error = %d\n", avePos, imgInfo.error);

    // int avePos, predictedPass;
    // float spd = 2.5;// = get_speed() 之后写成由speed动态选择前瞻的模式
    // predictedPass = min(spd * 2, imgInfo.top - 4); //之后根据实际情况写出关系式
    // if (predictedPass < 0)
    // {
    //     for(int i = imgInfo.bottom; i>imgInfo.top ; --i)
    //     {
    //         avePos += rowInfo[i].midLine;
    //     }
    //     avePos /= imgInfo.bottom - imgInfo.top;
    // }
    // else
    // {
    //     avePos = (rowInfo[predictedPass].midLine + rowInfo[predictedPass + 1].midLine + rowInfo[predictedPass + 2].midLine) / 3;
    // }
    // imgInfo.error = avePos-(WIDTH/2);

}
/**
 * @description: 判断左/中/右线的连续性
 * @param {uint8_t} select_top
 * @param {uint8_t} select_bottom
 * @param {LineTypeEnum} type
 * @return {*}
 */
uint8_t judge_lineContinuity(uint8_t select_top, uint8_t select_bottom, LineTypeEnum type)
{
    int delta;
    switch (type)
    {
        case LEFT:
            for (int j = select_top; j < select_bottom; ++j)//从第10行开始，防止下面提早断掉，影响判断
            {
                delta = rowInfo[j + 1].leftLine - rowInfo[j].leftLine;       //left线的偏差
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
                delta = rowInfo[j + 1].midLine - rowInfo[j].midLine;       //left线的偏差
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
                delta = rowInfo[j + 1].rightLine - rowInfo[j].rightLine;       //left线的偏差
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

float get_curvature(uint8_t select_top, uint8_t select_bottom, LineTypeEnum type)
{
    uint8_t select_mid = (select_top + select_bottom) / 2;
    float ret;
    switch (type)
    {
        case LEFT:
            ret = calc_curvature(select_top, rowInfo[select_top].leftLine,
                                      select_mid, rowInfo[select_mid].leftLine,
                                      select_bottom, rowInfo[select_bottom].leftLine);
            break;
        case RIGHT:
            ret = calc_curvature(select_top, rowInfo[select_top].rightLine,
                                      select_mid, rowInfo[select_mid].rightLine,
                                      select_bottom, rowInfo[select_bottom].rightLine);
            break;
        case MID:
            ret = calc_curvature(select_top, rowInfo[select_top].midLine,
                                      select_mid, rowInfo[select_mid].midLine,
                                      select_bottom, rowInfo[select_bottom].midLine);
            break;
        default:
            break;
    }
    return ret;
}

/**
 * @description: 三点计算曲率
 * @param {uint8_t} x1
 * @param {uint8_t} y1
 * @param {uint8_t} x2
 * @param {uint8_t} y2
 * @param {uint8_t} x3
 * @param {uint8_t} y3
 * @return {float}
 */
float calc_curvature(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3)
{
    float K;
    int S_of_ABC = ((int)(x2 - x1) * (y3 - y1) - (int)(x3 - x1) * (y2 - y1)) / 2;
    //面积的符号表示方向
    int q1 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    float AB = sqrt(q1);
    q1 = (x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2);
    float BC = sqrt(q1);
    q1 = (x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1);
    float AC = sqrt(q1);
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
 * @param {uint8_t} select_top 范围起点（不包括在内）
 * @param {uint8_t} select_bottom 范围终点（包括在内）
 * @param {LineTypeEnum} type 补线的对象类型 {LEFT/MID/RIGHT}
 * @return {*}
 */
void add_line(float k, float b, uint8_t select_top, uint8_t select_bottom, LineTypeEnum type)
{
    switch (type)
    {
        case LEFT:
            for(int i = select_top + 1; i <= select_bottom; ++i)
            {
                rowInfo[i].leftLine = k * i + b;
            }
            break;
        case MID:
            for(int i = select_top + 1; i <= select_bottom; ++i)
            {
                rowInfo[i].midLine = k * i + b;
            }
            break;
        case RIGHT:
            for(int i = select_top + 1; i <= select_bottom; ++i)
            {
                rowInfo[i].rightLine = k * i + b;
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
 * @param {uint8_t} select_top 选定范围的开始（高度范围）
 * @param {uint8_t} select_bottom 选定范围的结束（高度范围）
 * @param {LeastSquaresObjectEnum} type 需要拟合的直线类型（左边界/右边界/中线）
 * @return {*} 无返回值，通过传入 $k$ 和 $b$ 得到直线参数
 */
void least_squares(float * k, float * b, uint8_t select_top, uint8_t select_bottom, LineTypeEnum type)
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
                sumxy += i * rowInfo[i].leftLine;
                sumy  += rowInfo[i].leftLine;
                sumy2 += rowInfo[i].leftLine * rowInfo[i].leftLine;
            }
            break;
        case MID:
            for (int i = select_top; i <= select_bottom; ++i)
            {
                sumx  += i; //i 即为x轴值
                sumx2 += i * i;
                sumxy += i * rowInfo[i].midLine;
                sumy  += rowInfo[i].midLine;
                sumy2 += rowInfo[i].midLine * rowInfo[i].midLine;
            }
            break;
        case RIGHT:
            for (int i = select_top; i <= select_bottom; ++i)
            {
                sumx  += i; //i 即为x轴值
                sumx2 += i * i;
                sumxy += i * rowInfo[i].rightLine;
                sumy  += rowInfo[i].rightLine;
                sumy2 += rowInfo[i].rightLine * rowInfo[i].rightLine;
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

inline float cosAOB(int xa, int ya, int xo, int yo, int xb, int yb)
{
    int dot;
    float vecA, vecB;
    dot = (xa - xo) * (xb - xo) + (ya - yo) * (yb - yo);
    vecA = sqrt( (xa - xo) * (xa - xo) + (ya - yo) * (ya - yo) );
    vecB = sqrt( (xb - xo) * (xb - xo) + (yb - yo) * (yb - yo) );
    return (float)dot/(vecA * vecB);
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
            sumY += rowInfo[i].midLine;
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += rowInfo[i].midLine;
        }
        averageX = sumX / (sumlines1 + sumlines2);     //x的平均值
        averageY = sumY / (sumlines1 + sumlines2);     //y的平均值
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (rowInfo[i].midLine - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (rowInfo[i].midLine - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;

    }
    else if (type == 1)     //拟合左线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += rowInfo[i].leftLine;
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += rowInfo[i].leftLine;
        }
        averageX = sumX / (sumlines1 + sumlines2);     //x的平均值
        averageY = sumY / (sumlines1 + sumlines2);     //y的平均值
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (rowInfo[i].leftLine - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (rowInfo[i].leftLine - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
    else if (type == 2)         //拟合右线
    {
        /**计算sumX sumY**/
        for (i = startline1; i < endline1; i++)
        {
            sumX += i;
            sumY += rowInfo[i].rightLine;
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += rowInfo[i].rightLine;
        }
        averageX = sumX / (sumlines1 + sumlines2);     //x的平均值
        averageY = sumY / (sumlines1 + sumlines2);     //y的平均值
        for (i = startline1; i < endline1; i++)
        {
            sumUp += (rowInfo[i].rightLine - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        for (i = startline2; i < endline2; i++)
        {
            sumUp += (rowInfo[i].rightLine - averageY) * (i - averageX);
            sumDown += (i - averageX) * (i - averageX);
        }
        if (sumDown == 0) parameterB = 0;
        else parameterB = sumUp / sumDown;
        parameterA = averageY - parameterB * averageX;
    }
}
