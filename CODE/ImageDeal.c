/*
 * @Author: ROY1994
 * @Date: 2022-05-10 17:23:16
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-13 22:03:59
 * @FilePath: \myImageDeal\ImageDeal.cpp
 * @Description:
 */
/*
 * @Author: ROY1994
 * @Date: 2022-02-04 14:01:30
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-07-03 14:13:56
 * @FilePath: \myImageDeal\ImageDeal.cpp
 */
#include "ImageDeal.h"
#include <stdint.h>
#include <stdio.h>
//#include "system.h"
//#include "vt100.h"

#ifdef DEBUG
#include "DEBUGDEFINE.h"
#endif

#define PIXEL(ROW_NUM, LINE) imageBin[ROW_NUM][rowInfo[ROW_NUM].LINE##Line]


#define GET_VARIANCE(VAR, LINE_TYPE, TOLERANT)\
do {\
    VAR = 0;\
    float pred_k =    (float)(rowInfo[HEIGHT - TOLERANT].LINE_TYPE##Line - rowInfo[imgInfo.top + TOLERANT].LINE_TYPE##Line)\
                    / (HEIGHT - TOLERANT - (imgInfo.top + TOLERANT));\
    float pred_b =    (float)rowInfo[(int)((HEIGHT + imgInfo.top) / 2)].LINE_TYPE##Line\
                    - pred_k * ((HEIGHT + imgInfo.top) / 2.0);\
    for (int row = HEIGHT - TOLERANT; row > (imgInfo.top + TOLERANT); row--)\
    {\
        VAR +=   (rowInfo[row].LINE_TYPE##Line - (int)(pred_k * row + pred_b))\
               * (rowInfo[row].LINE_TYPE##Line - (int)(pred_k * row + pred_b));\
    }\
}while(0)\


#if defined (DEBUG)
DebugVaribleTypedef debugVar;
#endif

#if defined (__ON_ROBOT__)
extern DebugDataTypedef DebugData;
#endif

int LineEdgeScanWindow_Cross = 3;  //ʮ��ɨ�߷�Χ
int LineEdgeScanWindow = 5;    //ֱ��������ֵ

uint8_t needExternL = 0;  //�Ƿ����ӳ���־
uint8_t needExternR = 0;  //�Ƿ����ӳ���־

extern uint8_t mt9v03x_image[120][188];//ԭ�Ҷ�ͼ
extern uint8_t imageBin[HEIGHT][WIDTH];//��ֵ��ͼ��
extern ConstDataTypeDef ConstData;
extern SystemDataTypedef SystemData;
extern long long picCount;

PixelTypedef leftLineSerial[HEIGHT<<1],rightLineSerial[HEIGHT<<1];//�����������������
int16_t leftLineSeriesCnt,rightLineSeriesCnt;//�����������������

RowInfoTypedef rowInfo[HEIGHT];//����Ϣ

int16_t regWidth[HEIGHT] = {17,18,18,19,19,21,22,22,23,24,27,28,28,28,30,31,33,33,35,36,37,38,39,40,42,42,44,46,46,48,49,50,52,52,54,54,56,58,58,60,61,62,64,64,66,67,68,69,71,71,73,74,75,77,79,80,80,81,83,84,85,86,88,88,90,91,93,94,95,96,97,98,99,101,102,103,105,105,107,107,109,110,111,113,114,115,115,117,117,119,119,121,122,123,124,125,127,127,129,130,131,133,133,135,136,137,139,139,141,141,143,144,145,146,148,149,150,151,152,153,};
//Ԥ�����ÿ�п�ȣ���ǿ���ã�

//���߼����ߵĴ���״̬�����Ҫ�ع�����xxxline[]��ֵMISS��
uint8_t allFindCnt = 0,allLostCnt = 0, leftLostCnt = 0, rightLostCnt = 0;//������ȫ�ҵ�������������ȫ��ʧ���������߶�ʧ���������߶�ʧ����
uint8_t leftDownJump, rightDownJump, leftUpJump, rightUpJump, leftDownStart, rightDownStart, leftUpStart, rightUpStart;
// ����/����/����/���Ϲյ�          ����/����/����/���ϱ߽������ʼ��
uint8_t leftSeriesBreak, rightSeriesBreak;

// int16_t ThreeForkCnt = 0; // 0-δ���� 1-����1��,Ŀǰ���������� 2-����2��,Ŀǰ�Ѿ���һ�� 3-����3��,Ŀǰ���������� 4-���������
                          //Ĭ��������ת

float parameterA, parameterB;//���������ѷ�����

ImgInfoTypedef imgInfo;//ͼ����������ṹ�壨����ع���������HEIGHT��С�Ŀ��ɽṹ�壩


uint8_t ForkLinePointx_l, ForkLinePointx_r, ForkLinePointy;//����㲹������
int forkDetectStartLine = 0, forkDetectSpecLine = 0;
char fork_flag_1 = 'F', fork_flag_2 = 'F', fork_flag_tot = 'F';
char fork_in_flag = 'F'; //�Ѿ���������־

EdgeJumpPointTypedef color_TogglePos_left[10], color_TogglePos_right[10];
// int color_TogglePos_left[10] = {0}, color_TogglePos_right[10] = {0};
int color_toggleCnt_left = 0, color_toggleCnt_right = 0;
// ��->��->��->��->�� �ĸ������(��ֹ���ܵ�Խ��)
uint8_t isCircle_flag_1 = 'F', isCircle_flag_2 = 'F', circle_in_flag = 'F';
// int CircleOrP_BlackBlock_Hrow = 0;
BlackBlockTopTypedef blackBlock;
uint8_t leftDownLost, rightDownLost;
int bigCircleTop = 0;


/**
 * @description: ͼ������ú���
 * @param {*}
 * @return {*}
 */
void img_process(void)
{
    params_init(); // ��ʼ������

    basic_searchLine(HEIGHT-1,HEIGHT-6); // �������6�н�����ȫɨ��
    advance_searchLine(HEIGHT-7); // ���������һ�еķ�Χ�ó�

#if defined(__ON_ROBOT__)

    if(stop_detect())
        SystemData.isStop = 'T';
//#define __BARN_OUT_ON__

#if defined(__BARN_OUT_ON__)

    if (SystemData.isBarnOut == 'F')
    {
        static uint8_t firstFlag = 'T';

        imgInfo.RoadType = Barn_Out;
        if(firstFlag == 'T')
        {
            start_integrating_angle();
            firstFlag = 'F';
        }
        barnOut_repairLine();
        if (check_yaw_angle() > DEGREE_67)
        {
            SystemData.isBarnOut = 'T';
            stop_interating_angle();
        }
    }
    else
    {
        road_judge();
    }
#else
    road_judge();
#endif

#else
    road_judge();
#endif


#ifdef DEBUG
    PRINT_ROADTYPE_INFO();
#endif

    //TODO
    advance_repairLine(); // ����
    advance_midLineFilter();

    get_error();
}

/**
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
    leftDownLost = 'F'; rightDownLost = 'F';
    ForkLinePointx_l = 0; ForkLinePointx_r = 0; ForkLinePointy = 0;
    blackBlock.posX = MISS;
    blackBlock.posY = MISS;
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

#if defined(__ON_ROBOT__)
//    reset_debugData();
#endif
}

/**
 * @description: ����ɨ�ߺ���(ǰn��)
 *               ��ȡͼ��������ߴ���������������꣬�м�ֵ��ͼ��Χ�Ȳ���
 * @param {*}
 * @return {*}
 */
void basic_searchLine(int bottom,int top)
{
    /**********************************��������*********************************/

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
 * @description: ������ɨ�� (����״̬)
 * @param {*}
 * @return {*}
 */
void series_searchLine(void)
{
    int dx[8] = {1, 1, 0,-1,-1,-1, 0, 1,};
    int dy[8] = {0,-1,-1,-1, 0, 1, 1, 1,};
    /**
     * ���� # ��ΧΪ
     *
     *   3 2 1
     *   4 # 0
     *   5 6 7
     *
     */
    int16_t start = HEIGHT - 1;
    int16_t x, y, tx, ty, s, SearchCompleteFlag, SearchDisruptFlag;

    //������ʼ�а׵�ƽ��λ��
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

    //��ȡ��������ʼλ��
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

    /*-------------------�����������������-------------------*/
    s = 0;
    x = leftLineSerial[leftLineSeriesCnt].x;
    y = start;
    SearchCompleteFlag = 0; // ������ɱ�־λ
    SearchDisruptFlag = 0;  // ������;���б�־λ

    while(1)
    {
        //ǿ�ƽ���,��ֹɨ��ͷ
        if(y <= imgInfo.top)
            break;
        // printf(" $%d %d$ \n", x, y);
        //��ʱ����תɨ��
        for( ; ; ++s)
        {
            s = (s + 8) % 8;
            // if(s == 8) s = 0;

            tx = x + dx[s];// ��ΧԪ�ص�����
            ty = y + dy[s];
            if(ty < 0 || ty >= HEIGHT || tx >= WIDTH)// ɨ���߽磬�������
            {
                SearchCompleteFlag = 1;
                break;
            }
            if(tx < 0)// �ڿ����ڣ��ϴε�xֵ���䣬ty�����ƶ�
            {
                SearchDisruptFlag = 1;
                y = ty;
                s =  (s - 1 + 8) % 8;
                break;
            }
            if(!imageBin[ty][tx])// ��ɫ���������䣩����x��y
            {
                SearchDisruptFlag = 0;//�˳�����״̬
                x = tx;
                y = ty;
                s =  (s - 2 + 8) % 8;
                break;
            }
        }

        if(SearchCompleteFlag == 1)// ��������˳�
        {
            break;
        }
        if(SearchDisruptFlag == 1)// �����жϼ���
        {
            continue;//
        }

        ++leftLineSeriesCnt;// �������ӵ�ǰ�ߵĳ���
        leftLineSerial[leftLineSeriesCnt].x = tx;
        leftLineSerial[leftLineSeriesCnt].y = ty;
    }

    /*-------------------�����������������-------------------*/
    //ע�⣬��ʱӦ����Ϊ˳ʱ�뷽������
    s = 4;
    x = rightLineSerial[rightLineSeriesCnt].x;
    y = start;
    SearchCompleteFlag = 0; // ������ɱ�־λ
    SearchDisruptFlag = 0;  // ������;���б�־λ

    while(1)
    {
        //ǿ�ƽ���,��ֹɨ��ͷ
        if(y <= imgInfo.top)
            break;
        // printf(" $%d %d$ \n", x, y);
        //˳ʱ����תɨ��
        for( ; ; --s)
        {
            s = (s + 8) % 8;
            // if(s == 8) s = 0;

            tx = x + dx[s];// ��ΧԪ�ص�����
            ty = y + dy[s];
            if(ty < 0 || ty >= HEIGHT || tx < 0)// ɨ���߽磬�������
            {
                SearchCompleteFlag = 1;
                break;
            }
            if(tx >= WIDTH)// �ڿ����ڣ��ϴε�xֵ���䣬ty�����ƶ�
            {
                SearchDisruptFlag = 1;
                y = ty;
                s =  (s + 1 + 8) % 8;
                break;
            }
            if(!imageBin[ty][tx])// ��ɫ���������䣩����x��y
            {
                SearchDisruptFlag = 0;
                x = tx;
                y = ty;
                s =  (s + 2 + 8) % 8;
                break;
            }
        }

        if(SearchCompleteFlag == 1)// ��������˳�
        {
            break;
        }
        if(SearchDisruptFlag == 1)// �����жϼ���
        {
            continue;
        }

        ++rightLineSeriesCnt;// �������ӵ�ǰ�ߵĳ���
        rightLineSerial[rightLineSeriesCnt].x = tx;
        rightLineSerial[rightLineSeriesCnt].y = ty;
    }
}

/**
 * @description: ��ȡ�յ����ʼ����� (��ʮ�ּ������յ�Ѱ������) (����״̬)
 * @param {uint8_t} select_top
 * @param {uint8_t} select_bottom
 * @return {*}
 */
void basic_getSpecialParams(uint8_t select_top, uint8_t select_bottom)
{
    int i;
    leftDownJump = MISS;
    leftUpJump = MISS;
    rightDownJump = MISS;
    rightUpJump = MISS;

    //Ѱ���·����
    for (i = select_bottom; i >= select_top; --i)//�����������ʼλ�ú�ȱʧ���
    {
        //���߲���
        if(rowInfo[i].leftStatus == EXIST && !leftDownStart)     //ɨ����
        {
            leftDownStart = i;
        }
        //���߲���
        else if(rowInfo[i].rightStatus == EXIST && !rightDownStart)    //ɨ����
        {
            rightDownStart = i;
        }
    }
    
    // leftLastDiff = abs(leftLine[leftDownStart - 1] - leftLine[leftDownStart]);
    // rightLastDiff = abs(rowInfo[rightDownStart - 1] - rowInfo[rightDownStart]);
    //Ѱ���²�յ�
    for (i = leftDownStart - 1; i > select_top; --i)
    {
        //ֱ���ж�һ����Ĳ�ֵ������ֵ�������Բ��Ǻܺ�
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
    
    //Ѱ���ϲ����
    for (i = select_top; i <= select_bottom; ++i)//�����������ʼλ�ú�ȱʧ���
    {
        //���߲���
        if(rowInfo[i].leftStatus == EXIST && !leftUpStart)     //ɨ����
        {
            leftUpStart = i;
        }  
        //���߲���
        if(rowInfo[i].rightStatus == EXIST && !rightUpStart)    //ɨ����
        {
            rightUpStart = i;
        }
    }
    //Ѱ���ϲ�յ�
    for (i = leftUpStart + 2; i < select_bottom; ++i)//��2�ܿ���ֹ�߸���
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


/**
 * @description: ����״̬
 * @return {*}
 */
void series_getSpecialParams(void)
{
    //��ʼ��ֱ��ȡ��������
    leftDownStart = leftLineSerial[1].y;
    rightDownStart = rightLineSerial[1].y;
    leftUpStart = leftLineSerial[leftLineSeriesCnt].y;
    rightUpStart = rightLineSerial[rightLineSeriesCnt].y;

    //Ĭ������¶���λ�������
    leftSeriesBreak = leftLineSeriesCnt;
    rightSeriesBreak = rightLineSeriesCnt;

    //prev4Ϊ����4��Ԫ��, next4Ϊ�Ҳ���ĸ�Ԫ��, nowΪ��ǰԪ��, leftMaxCosi rightMaxCosi Ϊ�н��������ֵ��Ӧ��Ԫ��
    int prev4 = 1, next4 = 1, leftMaxCosi, rightMaxCosi;
    //p1 p2 Ϊ ��������ģ��, cosTmp Ϊ����ֵ, leftMaxCos rightMaxCos Ϊ�н��������ֵ
    float cosTmp, leftMaxCos = -1.1, rightMaxCos = -1.1;


    // ����
    for (int i = 1; i <= leftLineSeriesCnt-4; ++i)
    {
        // �����ߵ��������������������õ�ǰ������Ԫ�ص�x��y��Ӧ����1����
        // �������������Ƴ���
        //      1. ���ڿ���
        //      2. ���ܴ����Ϲյ�

        if(i <= leftSeriesBreak)// �ڶ���֮��
        {
            //�ƶ�prev4��next4
            prev4 = max(1,i-4);
            next4 = min(leftSeriesBreak,i+4);
            //�����ж�
            if(abs(leftLineSerial[next4].x - leftLineSerial[next4 + 1].x) > 1 ||
            abs(leftLineSerial[next4].y - leftLineSerial[next4 + 1].y) > 1)
            {
                leftSeriesBreak = next4;
            }

            //����˿�����ֵ
            cosTmp = cosAOB(leftLineSerial[prev4].x, leftLineSerial[prev4].y,
                            leftLineSerial[i].x, leftLineSerial[i].y,
                            leftLineSerial[next4].x, leftLineSerial[next4].y);
            //����
            if(cosTmp > leftMaxCos)
            {
                leftMaxCosi = i;
                leftMaxCos = cosTmp;
            }
        }
        else//�ڶ���֮��
        {
            //��һ�ε�������, Ҫ��λmaxֵ, ˳������յ�
            if(i == leftSeriesBreak + 1)
            {
                leftDownJump = leftLineSerial[leftMaxCosi].y;

                leftMaxCos = -1.1;
            }
            //�ƶ�prev4��next4
            prev4 = max(leftSeriesBreak+1,i-4);
            next4 = min(leftLineSeriesCnt,i+4);
            //����˿�����ֵ
            cosTmp = cosAOB(leftLineSerial[prev4].x, leftLineSerial[prev4].y,
                            leftLineSerial[i].x, leftLineSerial[i].y,
                            leftLineSerial[next4].x, leftLineSerial[next4].y);
            //����
            if(cosTmp > leftMaxCos)
            {
                leftMaxCosi = i;
                leftMaxCos = cosTmp;
                // printf("LD %d: %f\n",i, leftMaxCos);
            }
        }

    }
    //���leftSeriesBreak���ֱ��, �����Ƿ�����Ϲյ�
    if(leftSeriesBreak != leftLineSeriesCnt)
    {
        leftUpJump = leftLineSerial[leftMaxCosi].y;
        // printf("LU: %f\n", leftMaxCos);
    }

    // ����
    prev4 = 1; next4 = 1;
    for (int i = 1; i <= rightLineSeriesCnt-4; ++i)
    {
        // �����ߵ��������������������õ�ǰ������Ԫ�ص�x��y��Ӧ����1����
        // �������������Ƴ���
        //      1. ���ڿ���
        //      2. ���ܴ����Ϲյ�

        if(i <= rightSeriesBreak)// �ڶ���֮��
        {
            //�ƶ�prev4��next4
            prev4 = max(1,i-4);
            next4 = min(rightSeriesBreak,i+4);
            //�����ж�
            if(abs(rightLineSerial[next4].x - rightLineSerial[next4 + 1].x) > 1 ||
            abs(rightLineSerial[next4].y - rightLineSerial[next4 + 1].y) > 1)
            {
                rightSeriesBreak = next4;
            }

            //����˿�����ֵ
            cosTmp = cosAOB(rightLineSerial[prev4].x, rightLineSerial[prev4].y,
                            rightLineSerial[i].x, rightLineSerial[i].y,
                            rightLineSerial[next4].x, rightLineSerial[next4].y);
            //����
            if(cosTmp > rightMaxCos)
            {
                rightMaxCosi = i;
                rightMaxCos = cosTmp;
            }
        }
        else//�ڶ���֮��
        {
            //��һ�ε�������, Ҫ��λmaxֵ, ˳������յ�
            if(i == rightSeriesBreak + 1)
            {
                rightDownJump = rightLineSerial[rightMaxCosi].y;
                rightMaxCos = -1.1;
            }
            //�ƶ�prev4��next4
            prev4 = max(rightSeriesBreak+1,i-4);
            next4 = min(rightLineSeriesCnt,i+4);
            //����˿�����ֵ
            cosTmp = cosAOB(rightLineSerial[prev4].x, rightLineSerial[prev4].y,
                            rightLineSerial[i].x, rightLineSerial[i].y,
                            rightLineSerial[next4].x, rightLineSerial[next4].y);
            //����
            if(cosTmp > rightMaxCos)
            {
                rightMaxCosi = i;
                rightMaxCos = cosTmp;
            }
        }

    }
    //���rightSeriesBreak���ֱ��, �����Ƿ�����Ϲյ�
    if(rightSeriesBreak != rightLineSeriesCnt)
    {
        rightUpJump = rightLineSerial[rightMaxCosi].y;
    }
}

/**
 * @description: �����·��յ������޲��·�ȱʧ�ߵĺ��� (����״̬)
 * @param {*}
 * @return {*}
 */
void basic_repairLine(void)//[x] �������ѡ��,��·�²ಹ��,midLine����,[x]��ֳ�����ж�ģ��)
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
    //����
    float left_curvature = get_curvature(leftDownStart, leftDownEnd,LEFT);
    if (left_curvature > -0.1 && left_curvature < 0.4 && leftDownStart - leftDownEnd >= 7)// && leftDownStart <= 32 )
    {
        least_squares(&k, &b, leftDownEnd, leftDownStart, LEFT);
        add_line(k, b, leftDownStart, imgInfo.bottom, LEFT);
    }

    //����
    float right_curvature = get_curvature(rightDownStart, rightDownEnd, RIGHT);
    if (right_curvature > -0.4 && right_curvature < 0.1 && rightDownStart - rightDownEnd >= 7)// && rightDownStart <= 32 
    {
        least_squares(&k, &b, rightDownEnd, rightDownStart, RIGHT);
        add_line(k, b, rightDownStart, imgInfo.bottom, RIGHT);
    }

    //�����޲��������
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
/**
 * @description: �� bottom ��ʼ��һ����Χ��ɨ��
 * @param {int} bottom
 * @return {*}
 */
void advance_searchLine(int bottom)
{
    uint8_t isFindLk = 'F';     //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
    uint8_t hasFindLk = 'F';    //�Ƿ��Թ��ҵ���һ֡ͼ��Ļ�׼��б��
    uint8_t isFindRk = 'F';     //ȷ���ޱ�б�ʵĻ�׼�б����Ƿ��ҵ��ı�־
    uint8_t hasFindRk = 'F';    //�Ƿ��Թ��ҵ���һ֡ͼ��Ļ�׼��б��
    float D_L = 0;              //�ӳ��������б��
    float D_R = 0;              //�ӳ����ұ���б��
    int firstLostL = 0;             //��ס�״��󶪱���
    int firstLostR = 0;             //��ס�״��Ҷ�����
    needExternR = 'T';            //��־λ��0
    needExternL = 'T';
    int tmpL = 0, tmpR = 0;
    for (int row = bottom; row > imgInfo.top; row--)  //ǰ5�д�����ˣ������55�е����趨�Ĳ��������top��
    {   //̫Զ��ͼ���ȶ���top�Ժ�Ĳ�����
        // picTemp = imageBin[row];

        EdgePointTypedef EdgePoint[2];  // 0��1��
        if (imgInfo.RoadType != Cross)
        {
            tmpL = (int)rowInfo[row + 1].rightLine - LineEdgeScanWindow;  //����һ���ұ���-5�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
            tmpR = (int)rowInfo[row + 1].rightLine + LineEdgeScanWindow;  //����һ���ұ���+5�ĵ������ȷ��ɨ������㣩
        }
        else
        {
            tmpL = (int)rowInfo[row + 1].rightLine - LineEdgeScanWindow_Cross;  //����һ���ұ���-5�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
            tmpR = (int)rowInfo[row + 1].rightLine + LineEdgeScanWindow_Cross;
        }

        LIMIT_L(tmpL);   //ȷ����ɨ�����䲢��������
        LIMIT_R(tmpR);  //ȷ����ɨ�����䲢��������


        basic_getJumpPointFromDet(imageBin[row], tmpL, tmpR, &EdgePoint[1], RIGHT);  //ɨ�ұ���

        tmpL = (int)rowInfo[row + 1].leftLine - LineEdgeScanWindow;  //����һ�������-5�ĵ㿪ʼ��ȷ��ɨ�迪ʼ�㣩
        tmpR = (int)rowInfo[row + 1].leftLine + LineEdgeScanWindow;  //����һ�������+5�ĵ������ȷ��ɨ������㣩

        LIMIT_L(tmpL);   //ȷ����ɨ�����䲢��������
        LIMIT_R(tmpR);  //ȷ����ɨ�����䲢��������


        basic_getJumpPointFromDet(imageBin[row],tmpL, tmpR, &EdgePoint[0], LEFT);

        if (EdgePoint[0].type == LOST)  //�����������߲��������䣬����10���㶼�ǰ׵�
        {
            rowInfo[row].leftLine = rowInfo[row + 1].leftLine;  //�������������һ�е���ֵ
        }
        else //���������
        {
            rowInfo[row].leftLine = EdgePoint[0].posX;  //��¼������
        }

        if (EdgePoint[1].type == LOST)  //��������ұ��߲���������
        {
            rowInfo[row].rightLine = rowInfo[row + 1].rightLine;  //�����ұ�������һ�е���ֵ
        }
        else //�ұ�������
        {
            rowInfo[row].rightLine = EdgePoint[1].posX;  //��¼������
        }

        rowInfo[row].leftStatus = EdgePoint[0].type;  //��¼�����Ƿ��ҵ����ߣ�����������
        rowInfo[row].rightStatus = EdgePoint[1].type;

        //********����ȷ����Щ������ı�Ե*********//
        if (EdgePoint[0].type == JUMP )  //�������ߴ�����
        {
            for (int col = (rowInfo[row].leftLine + 1); col <= (rowInfo[row].rightLine - 1); ++col)  //���ұ���֮������ɨ��
            {
                if (*(imageBin[row] + col) == 0 && *(imageBin[row] + col + 1) != 0)
                {
                    rowInfo[row].leftLine = col;  //�����һ������ߵ��ұ��кڰ�������Ϊ���Ա���ֱ��ȡ��
                    rowInfo[row].leftStatus = EXIST;
                    break;
                }
                else if (*(imageBin[row] + col) != 0)  ///һ�����ְ׵���ֱ������
                    break;
                else if (col == (rowInfo[row].rightLine - 1))
                {  //��������ұ��ߵ����ұߣ�����Ϊ�Ǿ��Ա���
                    rowInfo[row].leftLine = col;
                    rowInfo[row].leftStatus = EXIST;
                    break;
                }
            }
        }
        if ((rowInfo[row].rightLine - rowInfo[row].leftLine) <= 7)  //ͼ�����޶�
        {
            imgInfo.top = row + 1;  //������б�7С�˺���ֱ�Ӳ�Ҫ��
            break;
        }
        if (EdgePoint[1].type == JUMP)
        {
            for (int col = (rowInfo[row].rightLine - 1); col >= (rowInfo[row].leftLine + 1); col--)
            {
                if ((*(imageBin[row] + col) == 0) && (*(imageBin[row] + col - 1) != 0))
                {
                    rowInfo[row].rightLine = col;  ////����ұ��ߵ���߻��кڰ�������Ϊ���Ա���ֱ��ȡ��
                    rowInfo[row].rightStatus = EXIST;
                    break;
                }
                else if (*(imageBin[row] + col) != 0)
                    break;
                else if (col == (rowInfo[row].leftLine + 1))
                {  //�����������ߵ�����ߣ�����Ϊ�Ǿ��Ա���
                    rowInfo[row].rightLine = col;
                    rowInfo[row].rightStatus = EXIST;
                    break;
                }
            }
        }

        //*************����ȷ���ޱ���************//
        uint8_t L_found_point = 0;
        uint8_t R_found_point = 0;
        int tmpRightLine = WIDTH - 2, tmpLeftLine = 1;

        if (EdgePoint[1].type == LOST && row > 10 && row < HEIGHT-10) //������ֵ��ޱ���
        {
            if (hasFindRk == 'F') //��һ֡ͼ��û���ܹ�����һ�׼��ѭ��
            {
                hasFindRk = 'T';      //����  һ֡ͼ��ֻ��һ�� ��ΪT
                firstLostR = row + 2; //
                for (int y = row + 1; y < LIMIT_H(row + 30); y++)
                {
                    if (rowInfo[y].rightStatus == EXIST) //���ޱ�����������  һ�㶼���бߵ�
                        R_found_point++;
                }
                if (R_found_point > 8) //�ҵ���׼б�ʱ�  ���ӳ�������ȷ���ޱ�   ���бߵĵ�������8
                {
                    D_R = ((float)(rowInfo[row + R_found_point].rightLine - rowInfo[row + 3].rightLine))
                        / ((float)(R_found_point - 3));
                                                        //��������Щ����������б��
                                                        //�ø��ޱ������ӳ��������׼
                    if (D_R >= 0)
                    {
                        isFindRk ='T';
                            //���б�ʴ���0  ��ô�ҵ��������׼��  ��Ϊ���λ���
                            //����һ���������б�ʴ���0 С��0�����Ҳ�����ӳ� û��Ҫ
                    }
                    else
                    {
                        isFindRk = 'F'; //û���ҵ������׼��
                        if (D_R < 0)
                            needExternR = 'F'; //�����־λ����ʮ�ֽǵ㲹��  ��ֹͼ�����õ�
                    }
                }
            }
            if (isFindRk == 'T')
            {
                tmpRightLine = rowInfo[firstLostR].rightLine - D_R * (firstLostR - row); //����ҵ��� ��ô�Ի�׼�����ӳ���
            }

            LIMIT_L(tmpRightLine); //�޷�
            LIMIT_R(tmpRightLine); //�޷�
            rowInfo[row].rightLine = tmpRightLine;

        }

        if (EdgePoint[0].type == LOST && row > 10 && row < HEIGHT-10) //����ͬ��  ��߽�
        {
            if (hasFindLk == 'F')
            {
                hasFindLk = 'T';
                firstLostL = row + 2;
                for (int y = row + 1; y < LIMIT_H(row + 30); ++y)
                {
                    if (rowInfo[y].leftStatus == EXIST)
                        L_found_point++;
                }
                if (L_found_point > 8) //�ҵ���׼б�ʱ�  ���ӳ�������ȷ���ޱ�
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

            LIMIT_L(tmpLeftLine); //�޷�
            LIMIT_R(tmpLeftLine); //�޷�
            rowInfo[row].leftLine = tmpLeftLine;
        }

        // if (rowInfo[row].leftStatus == LOST && rowInfo[row].rightStatus == LOST)
        //         imgInfo.allLostCnt++; //Ҫ�����Ҷ��ޱߣ�������+1

        LIMIT_L(rowInfo[row].leftLine);  //�޷�
        LIMIT_R(rowInfo[row].leftLine);  //�޷�
        LIMIT_L(rowInfo[row].rightLine); //�޷�
        LIMIT_R(rowInfo[row].rightLine); //�޷�

        rowInfo[row].width = rowInfo[row].rightLine - rowInfo[row].leftLine;
        rowInfo[row].midLine = (rowInfo[row].rightLine + rowInfo[row].leftLine) / 2;


        if (rowInfo[row].width <= WIDTH / 10) //����ȷ�����Ӿ���
        {
            imgInfo.top = row + 1;
            break;
        }
        else if (rowInfo[row].rightLine <= 10 || rowInfo[row].leftLine >= WIDTH - 10)
        {
            imgInfo.top = row + 1;
            break;
        } //��ͼ����С��0�������ұߴﵽһ��������ʱ������ֹѲ��

#ifdef DEBUG
    // printf("rowInfo[%3d].leftLine = %4d      Status = %3d   ", row, rowInfo[row].leftLine, rowInfo[row].leftStatus);

    // printf("rowInfo[%3d].rightLine = %4d      Status = %3d\n", row, rowInfo[row].rightLine, rowInfo[row].rightStatus);
#endif
    }
    return;
}
#undef LIMIT_L
#undef LIMIT_R
#undef LIMIT_H

/**
 * @description: ��ȡ�����е����ұ��ߺ�����
 * @param {uint8_t} *row
 * @param {int} L
 * @param {int} R
 * @param {EdgePointTypedef} *Q
 * @param {LineTypeEnum} type
 * @return {*}
 */
void basic_getJumpPointFromDet(uint8_t *row, int L,int R, EdgePointTypedef *Q, LineTypeEnum type) //��һ��������Ҫ���ҵ�����
{
    if (type == LEFT) //ɨ�������
    {
        for (int i = R; i >= L; --i)
        {
            if (*(row + i)&& !(*(row + i - 1))) //�ɰױ��
            {
                Q->posX = i - 1;  //��¼�����
                Q->type = EXIST; /////��ȷ����
                break;
            }
            else if (i == (L + 1)) //����ɨ�����Ҳû�ҵ�
            {
                if (*(row + (L + R) / 2)) //����м��ǰ׵�
                {
                    Q->posX = (L + R) / 2; //��Ϊ��������е�
                    Q->type = LOST;          /////����ȷ�������м�Ϊ�ף���Ϊû�б�
                    break;
                }
                else ////����ȷ�������м�Ϊ��
                {
                    Q->posX = R;  //����м��Ǻڵ�
                    Q->type = JUMP; //�����ֱ�����ֵ����Ϊ�Ǵ�����
                    break;
                }
            }
        }
    }
    else if (type == RIGHT)
    {
        for (int i = L; i <= R; ++i)
        {
            if (*(row + i) && !(*(row + i + 1))) //�ɺڱ��
            {
                Q->posX = i + 1;  //��¼�ұ���
                Q->type = EXIST; /////��ȷ����
                break;
            }
            else if (i == (R - 1)) //����ɨ�����Ҳû�ҵ�
            {
                if (*(row + (L + R) / 2)) //����м��ǰ׵�
                {
                    Q->posX = (L + R) / 2; //��Ϊ�ұ������е�
                    Q->type = LOST;        //����ȷ�������м�Ϊ�ף���Ϊû�б�
                    break;
                }
                else ////����ȷ�������м�Ϊ��
                {
                    Q->posX = L;  //����м��Ǻڵ�
                    Q->type = JUMP; //�ұ���ֱ�����ֵ����Ϊ�Ǵ�����
                    break;
                }
            }
        }
    }
}

/**
 * @description: ���ڳ�����������²���
 * @return {*}
 */
void advance_repairLine(void) /////�����ӳ��߲�����ȷ������
{
    int FTSite = 0,TFSite = 0;
    float kL = 0.0, kR = 0.0;

    if (imgInfo.RoadType == Circle_L && imgInfo.CircleStatus == CIRCLE_OFF)
        TFSite = HEIGHT - 5;
    if (needExternL == 'T')
    {
        for (int row = HEIGHT-6; row >= (imgInfo.top + 5); row--)
        //�ӵ����п�ʼ����ɨɨ��������������   ��β���
                        //������ֻ��һ��
        {
            if (rowInfo[row].leftStatus == LOST) //���������߽�ûɨ����ɨ�����ǰ�ɫ��˵������û����߽��
            {
                if (rowInfo[row + 1].leftLine >= WIDTH - 10) //�����߽�ʵ����̫�ұ�
                {
                    imgInfo.top = row + 1;
                    break; //ֱ�����������������
                }

                while (row >= (imgInfo.top + 4)) //��ʱ��ûɨ������
                {
                    row--; //��������ɨ
                    if (rowInfo[row].leftStatus == EXIST &&
                        rowInfo[row - 1].leftStatus == EXIST &&
                        rowInfo[row - 2].leftStatus == EXIST &&
                        rowInfo[row - 2].leftLine > 0 &&
                        rowInfo[row - 2].leftLine < WIDTH - 10) //���ɨ�����г����˲��ұ��������������ж�����߽�㣨��߽��ڿհ��Ϸ���
                    {
                        FTSite = row - 2; //�ѱ�������ĵڶ��д���FTsite
                        break;
                    }
                }

                kL = ((float)(rowInfo[FTSite].leftLine - rowInfo[TFSite].leftLine)) / ((float)(FTSite - TFSite)); //��߽��б�ʣ��е������/�е������
                if (FTSite > imgInfo.top)
                {
                    for (int y = TFSite; y >= FTSite; y--)
                         //�ӵ�һ��ɨ������߽������ڶ��е����꿪ʼ����ɨֱ���հ��Ϸ�����߽��������ֵ
                    {
                        rowInfo[y].leftLine = (int)(kL * ((float)(y - TFSite))) + rowInfo[TFSite].leftLine;
                        //�����ڼ�Ŀհ״����ߣ���б�ߣ���Ŀ���Ƿ���ͼ����
                    }
                }
            }
            else
                TFSite = row + 2; //���ɨ���˱��е���߽磬���д��������棬����б�ʣ�
        }
    }

    if (imgInfo.RoadType == Circle_R && imgInfo.CircleStatus == CIRCLE_OFF)
        TFSite = HEIGHT - 5;
    if (needExternR == 'T')
    {
        for (int row = HEIGHT-6; row >= (imgInfo.top + 5); row--) //�ӵ����п�ʼ����ɨɨ��������������
        {
            if (rowInfo[row].rightStatus == LOST) //��������ұ߽�ûɨ����ɨ�����ǰ�ɫ��˵������û���ұ߽�㣬���Ǵ��������ڵ�
            {
                if (rowInfo[row + 1].rightLine <= 10) //����ұ߽�ʵ����̫���
                {
                    imgInfo.top = row + 1; //ֱ������
                    break;
                }
                while (row >= (imgInfo.top + 4)) //��ʱ��ûɨ��������������
                {
                    row--;
                    if (rowInfo[row].rightStatus == EXIST &&
                        rowInfo[row - 1].rightStatus == EXIST &&
                        rowInfo[row - 2].rightStatus == EXIST &&
                        rowInfo[row - 2].rightLine < WIDTH-1 &&
                        rowInfo[row - 2].rightLine > 10)
                        //���ɨ�����г����˲��ұ��������������ж�����߽�㣨��߽��ڿհ��Ϸ���
                    {
                        FTSite = row - 2; // �ѱ�������ĵڶ��д���FTsite
                        break;
                    }
                }

                kR = ((float)(rowInfo[FTSite].rightLine - rowInfo[TFSite].rightLine)) / ((float)(FTSite - TFSite));
                    //�ұ߽��б�ʣ��е������/�е������
                if (FTSite > imgInfo.top)
                {
                    for (int y = TFSite; y >= FTSite; y--) //�ӵ�һ��ɨ�����ұ߽������ڶ��е����꿪ʼ����ɨֱ���հ��Ϸ����ұ߽��������ֵ
                    {
                        rowInfo[y].rightLine = (int)(kR * ((float)(y - TFSite))) + rowInfo[TFSite].rightLine;
                            //�����ڼ�Ŀհ״����ߣ���б�ߣ���Ŀ���Ƿ���ͼ����
                    }
                }
            }
            else
                TFSite = row + 2; //������е��ұ߽��ҵ��ˣ���Ѹ�������ڶ��������͸�TFsite
        }
    }
    if(needExternL == 'T' || needExternR == 'T')
    {
        for (int row = HEIGHT-1; row >= imgInfo.top; row--)
        {
            rowInfo[row].midLine = (rowInfo[row].leftLine + rowInfo[row].rightLine) / 2; //ɨ�����������һ�龭�Ż�֮����м�ֵ����
            rowInfo[row].width = rowInfo[row].rightLine-rowInfo[row].leftLine; //���Ż�֮��Ŀ�ȴ���
        }
    }
}

//���ֶ��ߵ�ʱ��  ����ȷ���ޱ��е�����
void advance_midLineFilter(void)
{
    float kR = 0.0;
    int y;
    for (int row = HEIGHT - 2; row >= imgInfo.top + 5; --row) //�ӿ�ʼλ��ֹͣλ
    {
        if(
            row <= HEIGHT - 15 &&
            rowInfo[row].leftStatus == LOST &&
            rowInfo[row].rightStatus == LOST &&
            rowInfo[row - 1].leftStatus == LOST &&
            rowInfo[row - 1].rightStatus == LOST
          ) //��ǰ�����Ҷ��ޱߣ�������ǰ105��   �˲�
        {
            y = row;
            while (y >= (imgInfo.top + 5))
            {
                y--;
                if (rowInfo[y].leftStatus == EXIST && rowInfo[y].rightStatus == EXIST) //Ѱ�����߶������ģ��ҵ��뱾������ľͲ�����
                {
                    kR = (float)(rowInfo[y - 1].midLine - rowInfo[row + 2].midLine) / (float)(y - 1 - row - 2); //��б��
                    int CenterTemp = rowInfo[row + 2].midLine;
                    int LineTemp = row + 2;
                    while (row >= y)
                    {
                        rowInfo[row].midLine = (int)(CenterTemp + kR * (float)(row - LineTemp)); //��б�ʲ�
                        row--;
                    }
                    break;
                }
            }
        }
        rowInfo[row].midLine = (rowInfo[row - 1].midLine + 2 * rowInfo[row].midLine) / 3; //��ƽ����Ӧ�û�Ƚϻ�  ��������������ƽ��
    }
}

/**
 * @description: ����ѡ��Χ�����ݶ�Ӧ�÷�Χ����,֧����������
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
 * @description: �������ڵĵ�·Ԫ��
 * @param {*}
 * @return {*}
 */
void road_judge(void)
{
    /**
     *
     * ˼·�ǶԵ�
     * ��Ŀǰ�յ��ж�׼ȷ�Ȳ���
     * ����û�á���
     *
     * s��ʹ�����rֵ�ж�
     * һ��ȫ�������
     * һ��˫�յ� ����
     *
     */
    leftDownLost = judge_lineBeginLost(LEFT);
    rightDownLost = judge_lineBeginLost(RIGHT);

    if (
        // imgInfo.RoadType != Cross       &&     //����������0  if������������Ҫ�����˳� �����������0
        imgInfo.RoadType != Circle_L    &&
        imgInfo.RoadType != Circle_R    &&
        imgInfo.RoadType != P_L         &&
        imgInfo.RoadType != P_R         &&
        imgInfo.RoadType != Slope
      )
    {
        imgInfo.RoadType = Road_None;
    }
    if (
        imgInfo.RoadType != Cross       &&
        imgInfo.RoadType != Circle_L    &&
        imgInfo.RoadType != Circle_R    &&
        imgInfo.RoadType != P_L         &&
        imgInfo.RoadType != P_R         &&
        imgInfo.RoadType != Slope       &&
        fork_in_flag != 'T'
      )
    {
        straight_detect();
    }

    straight_speedUpDetect();

    if(
        imgInfo.RoadType != Cross       &&
        imgInfo.RoadType != P_L         &&
        imgInfo.RoadType != P_R         &&
        imgInfo.RoadType != Circle_L    &&
        imgInfo.RoadType != Circle_R
      )
    {
        fork_detect();
    }

    if (
        imgInfo.RoadType != Circle_L    &&
        imgInfo.RoadType != Circle_R    &&
        imgInfo.RoadType != P_L         &&
        imgInfo.RoadType != P_R         &&
        imgInfo.RoadType != Slope       &&
        imgInfo.RoadType != Fork_In     &&
        imgInfo.RoadType != Fork_Out    &&
        fork_in_flag != 'T'
       )
    {
        cross_detect();
    }


    if (
        imgInfo.RoadType != Cross       &&
        imgInfo.RoadType != Fork_In     &&
        imgInfo.RoadType != Fork_Out    &&
        fork_in_flag != 'T'
       )
    {
        circle_judge_1(); // �����жϺ���1
        circle_judge_2(); // �����жϺ���2
        bigCircleTop = get_circleTop(); //��ȡ�������ϱ�Ե
#ifdef DEBUG
        PRINT_CIRCLE_DETECT_FLAG_INFO();
#endif
        if(imgInfo.RoadType != Circle_L && imgInfo.RoadType != Circle_R)
        {
            p_detect();
        }
        if(imgInfo.RoadType != P_L && imgInfo.RoadType != P_R)
        {
            circle_detect();
        }
    }


    // TODO
    p_repairLine();
    circle_repairLine();
    fork_repairLine();

    return ;
}

void straight_detect(void)
{
    float midK = 0, sum = 0, variance = 0;//����б��, ���, ��������Ϸ���
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
    float variance = 0;  //�������м䷽��
    for (int row = HEIGHT - 5; row > imgInfo.top + 1; --row)
    {
        sum += (rowInfo[row].midLine - (WIDTH/2)) * (rowInfo[row].midLine - (WIDTH/2));
    }
    variance = (float)sum / (HEIGHT - 5 - imgInfo.top - 1);

    if (variance < (float)ConstData.kImageStraightLineSpeedUpVarianceTh && imgInfo.top <= 22
        && imgInfo.RoadType != Slope
        && imgInfo.RoadType != Fork_In
        && imgInfo.RoadType != Fork_Out
        && fork_in_flag != 'T'
        )
    {
        imgInfo.straight_needSpeedUP = 'T';
    }
    else
        imgInfo.straight_needSpeedUP = 'F';
}


/**
 * @description: �������ɨ��(����)
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
 * @description: ʮ�ּ��
 * @param {*}
 * @return {*}
 */
void cross_detect(void)
{
    int I = 0, U = 0;
    int startRow = (int)(HEIGHT * 2 / 3);
    startRow = max(startRow, imgInfo.top + 2);
    for (int row = startRow; row < HEIGHT; ++row)
    {
        if(rowInfo[row].leftStatus == LOST && rowInfo[row].rightStatus == LOST) ++I;
        if(rowInfo[row].leftStatus == LOST || rowInfo[row].rightStatus == LOST) ++U;
    }
    if((float)((float)I/U) > ConstData.kImageCrossIOUth)
        imgInfo.RoadType = Cross;
}


/**
 * @description: ������
 * @param {*}
 * @return {*}
 */
void fork_detect()
{
    uint8_t * PicTemp = NULL;

    // ��һ���� �жϽǶ�
    for (int row = HEIGHT - 10; row > (imgInfo.top + 6); row--) //��ֹrow���
    {
        if ((rowInfo[row].rightStatus == EXIST && rowInfo[row + 1].rightStatus == EXIST) ||
            (rowInfo[row].leftStatus == EXIST && rowInfo[row + 1].leftStatus == EXIST)) //�������ʱ��һ��ῴ����������120*��Բ��
        {
            if  ((

                    (rowInfo[row].leftLine - rowInfo[row - 6].leftLine) > 2 &&
                    (rowInfo[row].leftLine - rowInfo[row - 6].leftLine) < 8 &&
                    (rowInfo[row].leftLine - rowInfo[row + 6].leftLine) > 2 &&
                    (rowInfo[row].leftLine - rowInfo[row + 6].leftLine) < 8)
                    || (//��ߵĽ�
                    (rowInfo[row - 6].rightLine - rowInfo[row].rightLine) > 3 &&
                    (rowInfo[row - 6].rightLine - rowInfo[row].rightLine) < 8 &&
                    (rowInfo[row + 6].rightLine - rowInfo[row].rightLine) > 2 &&//�ұߵĽ� ����һ���Ϳ���  ��Ϊ������������©��
                    (rowInfo[row + 6].rightLine - rowInfo[row].rightLine) < 8

                ))
            {
                fork_flag_1 = 'T'; //��ʾ���˵Ľǵ������ҵ�
                forkDetectStartLine = row; //��¼��һ�����������
                break;
            }

            else
            {
                fork_flag_1 = 'F'; //û�ҵ�
            }
        }
    }

    int wide = 0;
    //�ڶ����� �Һ�ɫ���ǿ�  ������õ����ͼ����Ϣ
    for (int row = forkDetectStartLine; row > imgInfo.top; row--) //  �ӵ�һ�����㿪ʼ��������
    {
        wide = 0;
        PicTemp = imageBin[row];
        for (int x = rowInfo[row].leftLine; x < WIDTH / 2; x++) //������ں�ɫ���ǿ�
        {
            if (((*(PicTemp + x + 1) == 0) && (*(PicTemp + x + 2) == 0) && *(PicTemp + x) != 0))
            //�ҵ���ɫ�ǿ�����
            {
                rowInfo[row].fork_L = x + 1; //��¼��ʱ����ڱ߽�
                break;
            }
            else
                rowInfo[row].fork_L = MISS; //û�ҵ������е�
        }

        for (int x = rowInfo[row].rightLine; x > WIDTH / 2; x--) //������ں�ɫ���ǿ�
        {
            if ((*(PicTemp + x) == 0) && (*(PicTemp + x - 1) == 0) && (*(PicTemp + x + 1) != 0))
            //�ҵ���ɫ�ǿ���ұ�
            {
                rowInfo[row].fork_R = x; //��¼��ʱ���Һڱ߽�
                break;
            }
            else
                rowInfo[row].fork_R = MISS; //û�ҵ������е�
        }

        for (int x = rowInfo[row].fork_L; x <= rowInfo[row].fork_R; x++)
        {
            if (rowInfo[row].fork_L == MISS || rowInfo[row].fork_R == MISS) //������е�ֵ��ôû�ҵ�
            {
                break;
            }
            else if (*(PicTemp + x) == 0) //������ں��Һ�֮��ĺڵ���
            {
                wide++; //�����ҵ����ǿ�ı��кڵ���
            }
        }
        rowInfo[row].fork_blackWidth = wide; //��¼������
        rowInfo[row].fork_black_k = (float)rowInfo[row].fork_blackWidth / rowInfo[row].width; //ͼ��ڵ����                                              //��0
    }

    //�ж��Ƿ�Ϊ����ĺ�ɫ���ǿ�
    for (int row = forkDetectStartLine; row >= (imgInfo.top + 5); row--)
    {
        if ((rowInfo[row].fork_blackWidth - rowInfo[row + 3].fork_blackWidth) >= 2 &&
            //��������ں��Һ�֮��ڵ����Ƚ϶� �������������ε���״
            //�ٺ�бʮ������
            rowInfo[row].fork_blackWidth > 40 &&
            rowInfo[row + 1].fork_blackWidth > 30 &&
            rowInfo[row - 1].fork_blackWidth > 30 &&
            rowInfo[row].fork_L < WIDTH / 2 && //�˳�бʮ��
            rowInfo[row].fork_R > WIDTH / 2)
        {

            ForkLinePointx_r = rowInfo[row].fork_R; //���ڲ��ߵĵ�
            ForkLinePointx_l = rowInfo[row].fork_L;
            ForkLinePointy = row;

            forkDetectSpecLine = row;           //��¼�ڶ������������
            if (forkDetectStartLine - forkDetectSpecLine > 10) // g
            {
                fork_flag_2 = 'T'; //������������������10   ���ж�Ϊ�뻷������  ���ڷ�����
                break;
            }
        }
        else
            fork_flag_2 = 'F';
    }

    // �ۺ�������Ϣ�ж�
    if ((fork_flag_1 == 'T' && fork_flag_2 == 'T') || (fork_flag_tot == 'T' && fork_flag_2 == 'T'))
        fork_flag_tot = 'T';
    else
        fork_flag_tot = 'F';

    // ϣ��������⵽Fork_In��ʱ��

    // ����״̬λ����:
    // RoadType         Fork_In --> Road_None/Striaght --> Fork_Out --> Road_None/Striaght
    //                     -------------------------------------
    // fork_in_flag        |                                   |-----------------
    //
    static char fork_in_flag_last = 'F', fork_out_flag_last = 'F';
    static long long picNum_in = -1;
    char fork_in_flag_now = 'F', fork_out_flag_now = 'F';
    if(fork_in_flag == 'F' || ( picNum_in != -1 && picCount - picNum_in <= ConstData.kImageForkInPicCnt))
    {
        if(fork_flag_tot == 'T' && imgInfo.RoadType != Fork_In)
        {
            fork_in_flag_now = 'T';
            imgInfo.RoadType = Fork_In;
        }
        if(fork_in_flag_last == 'T' && fork_in_flag_now == 'F')
        {
            picNum_in = picCount;
            fork_in_flag = 'T';
            fork_in_flag_last = 'F';
        }
        else
        {
            fork_in_flag_last = fork_in_flag_now;
        }
    }
    else // fork_in_flag == 'T'
    {
        if (
            fork_flag_tot == 'T'
            && imgInfo.RoadType != Fork_Out
            && picNum_in != -1
            && picCount - picNum_in >= ConstData.kImageForkInOutPicCnt
           )
        {
            fork_out_flag_now = 'T';
            imgInfo.RoadType = Fork_Out;
        }
        if(fork_out_flag_last == 'T' && fork_out_flag_now == 'F')
        {
            picNum_in = -1;
            fork_in_flag = 'F';
            fork_out_flag_last = 'F';
        }
        else
        {
            fork_out_flag_last = fork_out_flag_now;
        }
    }
#ifdef DEBUG
    PRINT_FORK_DETECT_FLAG_INFO();
#endif

}

/**
 * @description: �����ߺ���
 * @return {*}
 */
void fork_repairLine()
{
    if(imgInfo.RoadType == Fork_In || imgInfo.RoadType == Fork_Out)
    {
        int LeftDown_x = 0;
        if(rowInfo[HEIGHT - 1].leftStatus == LOST || PIXEL(HEIGHT - 1, left) == 0)
        {
            LeftDown_x = rowInfo[HEIGHT - 1].leftLine;
        }
        float k = (float)(ForkLinePointx_r - LeftDown_x) / (ForkLinePointy - (HEIGHT - 1));
        float b = (float)(LeftDown_x - k * (HEIGHT - 1));
        add_line(k, b, ForkLinePointy - 1, HEIGHT - 1, LEFT);
        recalc_line(ForkLinePointy - 1, HEIGHT - 1, LEFT);
    }
}





/**
 * @description: �뻷���жϺ���1 �о�Ϊ���ߺڰ׽�����
 * @return {*}
 */
void circle_judge_1(void)
{
    color_toggleCnt_left = 0, color_toggleCnt_right = 0; // tmp
    isCircle_flag_1 = 'F'; // tmp

    for(int row = HEIGHT - 5; row > imgInfo.top + 2; row--)
    {
        // ���߽����ɫ���
        // if ((!imageBin[row][rowInfo[row].leftLine] && !imageBin[row - 1][rowInfo[row - 1].leftLine]) ||
        //     (imageBin[row][rowInfo[row].leftLine] && imageBin[row - 1][rowInfo[row - 1].leftLine]))
        // {
        //     continue ;
        // }

        if (!imageBin[row][rowInfo[row].leftLine] && imageBin[row - 1][rowInfo[row - 1].leftLine])
        {
            ++color_toggleCnt_left;
            color_TogglePos_left[color_toggleCnt_left].posY = row;
            color_TogglePos_left[color_toggleCnt_left].type = BlackToWhite;
        }
        else if (imageBin[row][rowInfo[row].leftLine] && !imageBin[row - 1][rowInfo[row - 1].leftLine])
        {
            ++color_toggleCnt_left;
            color_TogglePos_left[color_toggleCnt_left].posY = row - 1;
            color_TogglePos_left[color_toggleCnt_left].type = WhiteToBlack;
        }

        //���߽����ɫ���
        // if ((!imageBin[row][rowInfo[row].rightLine] && !imageBin[row - 1][rowInfo[row - 1].rightLine]) ||
        //     (imageBin[row][rowInfo[row].rightLine] && imageBin[row - 1][rowInfo[row - 1].rightLine]))
        // {
        //     continue ;
        // }

        if (!imageBin[row][rowInfo[row].rightLine] && imageBin[row - 1][rowInfo[row - 1].rightLine])
        {
            ++color_toggleCnt_right;
            color_TogglePos_right[color_toggleCnt_right].posY = row;
            color_TogglePos_right[color_toggleCnt_right].type = BlackToWhite;
        }
        else if (imageBin[row][rowInfo[row].rightLine] && !imageBin[row - 1][rowInfo[row - 1].rightLine])
        {
            ++color_toggleCnt_right;
            color_TogglePos_right[color_toggleCnt_right].posY = row - 1;
            color_TogglePos_right[color_toggleCnt_right].type = WhiteToBlack;
        }
    }

    // if(color_toggleCnt_left) color_toggleCnt_left--;
    // if(color_toggleCnt_right) color_toggleCnt_right--;

    if ((color_toggleCnt_left - color_toggleCnt_right >= 3 ) ||
        (color_toggleCnt_left - color_toggleCnt_right <= -3))
        isCircle_flag_1 = 'T';
    else
        isCircle_flag_1 = 'F';
}
PixelTypedef PointSerial[HEIGHT<<1]; // TODO �����Ƶ�������
int16_t SerialCnt = 0;//��������������


/**
 * @description: �뻷���жϺ���2 �о�Ϊ�����ɫ��Ȧ
 *               ͬʱΪ�ж�P���ṩ����Բ����ߵ�����
 * @return {*}
 */
void circle_judge_2(void)
{
    isCircle_flag_2 = 'F';

    // SerialCnt = 0;
    // int16_t SerialCnt = 0;//��������������
    blackBlock.posY = HEIGHT - 1;

    PixelTypedef start;

    const int dx[8] = {1, 1, 0,-1,-1,-1, 0, 1,};
    const int dy[8] = {0,-1,-1,-1, 0, 1, 1, 1,};
    /**
     * ���� # ��ΧΪ
     *
     *   3 2 1
     *   4 # 0
     *   5 6 7
     *
     */
#ifdef DEBUG
    printf("leftDownLost: %c    rightDownLost: %c\n", leftDownLost, rightDownLost);
#endif
    int16_t x, y, tx, ty, s, SearchCompleteFlag = 0, SearchDisruptFlag = 0;
    if (
        color_toggleCnt_left - color_toggleCnt_right >= 3 // ��ζ�Ż������󻷵�
        && (
            (leftDownLost == 'F' && color_TogglePos_left[2].type == WhiteToBlack)
            ||
            (leftDownLost == 'T' && color_TogglePos_left[2].type == BlackToWhite)
           ) //TODO
        )
    {
        int16_t SerialCnt_left = 0;
        // ��¼��ʼ��

        start.y = leftDownLost == 'T' ? color_TogglePos_left[1].posY : color_TogglePos_left[2].posY;
        start.x = rowInfo[start.y].leftLine;
        SerialCnt_left++;
        PointSerial[SerialCnt_left] = start;
        // ��ʼ������ɨ��
        s = 5;
        x = start.x;
        y = start.y;
        SearchCompleteFlag = 0; // ������ɱ�־λ

        // ��������Ƿ�Ϊ���ĺڿ���ߵ�
        if (blackBlock.posY > start.y)
        {
            blackBlock.posY = start.y;
            blackBlock.posX = start.x;
        }

        while(1)
        {
            // ǿ�ƽ���,��ֹɨ��ͷ
            int tmpTop = imgInfo.top + 2, s_cnt = 0;
            if( color_toggleCnt_left >= 4 &&
                color_TogglePos_left[4].type == WhiteToBlack &&
                tmpTop < color_TogglePos_left[4].posY)
            {
                tmpTop = color_TogglePos_left[4].posY;
            }
            if(y <= tmpTop || SerialCnt_left >= (HEIGHT<<1)) // С��ɨ����ֹ������һ��ͻ���ȥ��ֵ
            {
                isCircle_flag_2 = 'F';
                break;
            }
            // printf(" $%d %d$ \n", x, y);
            //��ʱ����תɨ��
            for( ; ; ++s, ++s_cnt)
            {
                s = (s + 8) % 8;
                // if(s == 8) s = 0;

                tx = x + dx[s];// ��ΧԪ�ص�����
                ty = y + dy[s];
                if(tx == start.x && ty == start.y)// ɨ����ʼ�㣬�������
                {
                    SearchCompleteFlag = 1;
                    break;
                }
                if(tx == 0 || tx == 1) continue; //��������ǿ�ƿ�����
                if(tx > WIDTH - 1 || ty > HEIGHT - 1 || ty < 0 || s_cnt > 8) // �����߽����ɨ����һȦ
                {
                    SearchDisruptFlag = 1;
                    break;
                }
                if(!imageBin[ty][tx])// ��ɫ�����䣩����x��y
                {
                    x = tx;
                    y = ty;

                    // ������³ɹ�, ���㵱ǰ�Ƿ�Ϊ���ĺڿ���ߵ�
                    // ����Ҫȡ���ұߵ���ߵ�, ��Ϊ��, ��ʱ��, ����ʹ�� > ��
                    if (blackBlock.posY > y)
                    {
                        blackBlock.posY = y;
                        blackBlock.posX = x;
                    }

                    s =  (s - 2 + 8) % 8; // ����һ��ֱ��
                    break;
                }
            }
            if(SearchCompleteFlag == 1)// ��������˳�
            {
                isCircle_flag_2 = 'T';
                break;
            }
            if(SearchDisruptFlag == 1)
            {
                isCircle_flag_2 = 'F';
                break;
            }
            ++SerialCnt_left;// �ǲ���
        }
    }
    else if (
            color_toggleCnt_left - color_toggleCnt_right <= -3 // ��ζ�Ż������һ���
            && (
                (rightDownLost == 'F' && color_TogglePos_right[2].type == WhiteToBlack)
                ||
                (rightDownLost == 'T' && color_TogglePos_right[2].type == BlackToWhite)
               )
            )//TODO
    {
        int16_t SerialCnt_right = 0;
        //��¼��ʼ��
        start.y = rightDownLost == 'T' ? color_TogglePos_right[1].posY : color_TogglePos_right[2].posY;
        start.x = rowInfo[start.y].rightLine;
        SerialCnt_right++;
        PointSerial[SerialCnt_right] = start;
        //��ʼ������ɨ��
        s = 5;
        x = start.x;
        y = start.y;
        SearchCompleteFlag = 0; // ������ɱ�־λ

        // ��������Ƿ�Ϊ���ĺڿ���ߵ�
        if (blackBlock.posY >= start.y)
        {
            blackBlock.posY = start.y;
            blackBlock.posX = start.x;
        }

        while(1)
        {
            //ǿ�ƽ���,��ֹɨ��ͷ
            int tmpTop = imgInfo.top + 2, s_cnt = 0;
            if( color_toggleCnt_right >= 4 &&
                color_TogglePos_right[4].type == WhiteToBlack &&
                tmpTop < color_TogglePos_right[4].posY)
            {
                tmpTop = color_TogglePos_right[4].posY;
            }
            if(y <= tmpTop || SerialCnt_right >= (HEIGHT<<1)) // С��ɨ����ֹ������һ��ͻ���ȥ��ֵ
            {
                isCircle_flag_2 = 'F';
                break;
            }
            // printf(" $%d %d$ \n", x, y);
            //��ʱ����תɨ��
            for( ; ; ++s, ++s_cnt)
            {
                s = (s + 8) % 8;
                // if(s == 8) s = 0;

                tx = x + dx[s];// ��ΧԪ�ص�����
                ty = y + dy[s];
                if(tx == start.x && ty == start.y)// ɨ����ʼ�㣬�������
                {
                    SearchCompleteFlag = 1;
                    break;
                }
                if(tx == WIDTH - 1 || tx == WIDTH - 2) continue; //��������ǿ�ƿ�����
                if(tx < 0 || ty > HEIGHT - 1 || ty < 0 || s_cnt > 8)
                {
                    SearchDisruptFlag = 1;
                    break;
                }
                if(!imageBin[ty][tx])// ��ɫ�����䣩����x��y
                {
                    x = tx;
                    y = ty;

                    // ������³ɹ�, ���㵱ǰ�Ƿ�Ϊ���ĺڿ���ߵ�
                    // ����Ҫȡ����ߵ���ߵ�, ��Ϊ�һ�, ��ʱ��, ����ʹ�� >= ��
                    if (blackBlock.posY >= y)
                    {
                        blackBlock.posY = y;
                        blackBlock.posX = x;
                    }

                    s =  (s - 2 + 8) % 8; // ����һ��ֱ��
                    break;
                }
            }
            if(SearchCompleteFlag == 1)// ��������˳�
            {
                isCircle_flag_2 = 'T';
                break;
            }
            if(SearchDisruptFlag == 1)
            {
                isCircle_flag_2 = 'F';
                break;
            }
            ++SerialCnt_right;// �ǲ���
#ifdef DEBUG
            PointSerial[SerialCnt_right].x = tx;
            PointSerial[SerialCnt_right].y = ty;
            SerialCnt = SerialCnt_right;
#endif
        }
    }
}

/**
 * @description: ��ȡ��״��ߵ�
 * @return {int} bigCircleTop_tmp ��״��ߵ�
 */
int get_circleTop(void)
{
    int bigCircleTop_tmp = MISS;
    if (isCircle_flag_2 == 'T')
    {
        for (int row = blackBlock.posY - 1; row >= 8; --row)
        {
            if (
                !imageBin[row - 1][blackBlock.posX]
                && !imageBin[row][blackBlock.posX]
                && imageBin[row + 1][blackBlock.posX]
                )
            {
                bigCircleTop_tmp = row + 1;
                break;
            }
        }
    }
    return bigCircleTop_tmp;
}


/**
 * @description: p�������жϺ���
 * @return {*}
 */
void p_detect(void)
{
    /*  ���� 1 (weak)
    *   �ж� imgInfo.top �����ཻ�ĺ�ɫ�߽����Ƿ���ڻ�õ� width ֵ
    *
    *   ���� 2 (weak)
    *   �жϴ� imgInfo.top �е���߽�, �ܷ��غ�ɫ�߽絽 imgInfo.top �����ཻ�ĺ�ɫ�ұ߽�
    *   .....................��....,......................................��....
    *
    *   ���� 3 (maybe)
    *   ���ǻ���/P���ĺ�ɫ����ߵ������и߶�����/�����ཻ�㸽���ĺڰ�״̬\
    *
    * __**-----___
    *              -- **
    * ____           - *
    *     /            \
    *    /              \
    * ---                \
    *                     \
    *                      \
    * /                     \
    *
    */
    // CircleOrP_BlackBlock_Hrow
    if (
            imgInfo.PStatus == P_NOT_FIND && isCircle_flag_2 == 'T'
            && color_toggleCnt_left - color_toggleCnt_right >= 3
            // && leftDownLost != 'T' && rightDownLost != 'T'
       ) // ��ཻ�������Ҳ��,����������P��
    {
        int p_variance_r = get_variance(blackBlock.posY + 5, HEIGHT - 10, RIGHT);
        // GET_VARIANCE(p_variance_r, right, 10);
#ifdef DEBUG
        PRINT_LINE_VARIANCE_INFO(p_variance_r);
#endif
        if (bigCircleTop <= imgInfo.top && imgInfo.top <= blackBlock.posY && p_variance_r < ConstData.kImageLineVarianceTh)
        {
            imgInfo.RoadType = P_L;
            imgInfo.PStatus = P_PASSING;
        }
    }
    else if (
            imgInfo.PStatus == P_NOT_FIND && isCircle_flag_2 == 'T'
            && color_toggleCnt_left - color_toggleCnt_right <= -3
            // && leftDownLost != 'T' && rightDownLost != 'T'
            ) // ��֮ ��P��
    {
        int p_variance_l = get_variance(blackBlock.posY + 5, HEIGHT - 10, LEFT);
        // GET_VARIANCE(p_variance_l, left, 10);
#ifdef DEBUG
        PRINT_LINE_VARIANCE_INFO(p_variance_l);
#endif
#if defined (__ON_ROBOT__)
        DebugData.PFlagInRange = (bigCircleTop <= imgInfo.top && imgInfo.top <= blackBlock.posY)? 'T':'F';
        DebugData.PFlagVariOK = (p_variance_l < ConstData.kImageLineVarianceTh)? 'T' : 'F';
#endif
        if (bigCircleTop <= imgInfo.top && imgInfo.top <= blackBlock.posY && p_variance_l < ConstData.kImageLineVarianceTh)
        {
            imgInfo.RoadType = P_R;
            imgInfo.PStatus = P_PASSING;
        }
    }

    // ================================================================== //
    // Old
    // ����Ҫ�����ĵ�
//     if (
//             imgInfo.PStatus == P_NOT_FIND && isCircle_flag_2 == 'T'
//             && color_toggleCnt_left - color_toggleCnt_right >= 3
//        ) // ��ཻ�������Ҳ��,����������P��
//     {
//         // �ȵõ��Ҳ�߽�ı��ʽ
//         float k = 0.0, b = 0.0;
//         least_squares(&k, &b, max(imgInfo.top + 5, HEIGHT - 70), HEIGHT - 5, RIGHT);
//         // ���ĺ�ɫ����ߵ������и߶� �� �����ཻ��
//         int Hcol = (int)(k * CircleOrP_BlackBlock_Hrow + b);
//         // ���õ���฽���ĺڰ�״̬, ������ڰ�ɫ, ����Ϊ����P��, ������һ�������ж�, ������Ϊ��P��
//         if (
//             !imageBin[CircleOrP_BlackBlock_Hrow][Hcol]           &&
//             !imageBin[CircleOrP_BlackBlock_Hrow][Hcol - 1]       &&
//             !imageBin[CircleOrP_BlackBlock_Hrow - 1][Hcol -1]    &&
//             !imageBin[CircleOrP_BlackBlock_Hrow - 1][Hcol - 2]   &&
//             !imageBin[CircleOrP_BlackBlock_Hrow - 2][Hcol - 4]
//            )
//         {
//             imgInfo.RoadType = P_L;
//             imgInfo.PStatus = P_PASSING;
//         }
//     }
//     else if (
//             imgInfo.PStatus == P_NOT_FIND && isCircle_flag_2 == 'T'
//             && color_toggleCnt_left - color_toggleCnt_right <= -3
//             ) // ��֮ ��P��
//     {
//         // ͬ�����߽�
//         float k = 0.0, b = 0.0;
//         least_squares(&k, &b, max(imgInfo.top + 5, HEIGHT - 70), HEIGHT - 5, LEFT);

//         // ���ĺ�ɫ����ߵ������и߶� �� �����ཻ��
//         int Hcol = (int)(k * CircleOrP_BlackBlock_Hrow + b);

// #ifdef DEBUG
//         printf("k: %f, b: %f\n", k, b);
//         debugVar.blackBlock_Hrow = CircleOrP_BlackBlock_Hrow;
//         debugVar.blackBlock_Hcol = Hcol;
// #endif
//         if (
//             !imageBin[CircleOrP_BlackBlock_Hrow][Hcol]           &&
//             !imageBin[CircleOrP_BlackBlock_Hrow][Hcol + 1]       &&
//             !imageBin[CircleOrP_BlackBlock_Hrow - 1][Hcol +1]    &&
//             !imageBin[CircleOrP_BlackBlock_Hrow - 1][Hcol + 2]   &&
//             !imageBin[CircleOrP_BlackBlock_Hrow - 2][Hcol + 4]
//         )
//         {
//             imgInfo.RoadType = P_R;
//             imgInfo.PStatus = P_PASSING;
//         }
//     }

    // �ж�Ϊ P ����, Ҫ�Գ��������⴦��
    // �ж��Ƿ񵽴� P_OUT_1 (��p��б������)ʱ��
    if (imgInfo.PStatus == P_PASSING)
    {
        if(imgInfo.RoadType  == P_L)
        {
            // �˷���ΪѰ�ҽǵ�, ���Ϸ����·��ĵ������Ҳ�
            for (int row = HEIGHT - 10; row > (imgInfo.top + 20); row--) //��ֹrow���
            {
                if (rowInfo[row].rightStatus == EXIST && rowInfo[row + 1].rightStatus == EXIST) //�ж����Ͻǵ��Ƿ����
                {
                    if (
                            (rowInfo[row - 4].rightLine - rowInfo[row].rightLine) > 2 &&
                            (rowInfo[row - 4].rightLine - rowInfo[row].rightLine) < 15 &&
                            (rowInfo[row + 6].rightLine - rowInfo[row].rightLine) > 2 &&
                            (rowInfo[row + 6].rightLine - rowInfo[row].rightLine) < 15
                       )//��ֵ��Ҫ����
                    {
                        imgInfo.PStatus = P_OUT_READY;
                        break;
                    }

                }
            }
        }
        else if (imgInfo.RoadType  == P_R)
        {
            for (int row = HEIGHT - 10; row > (imgInfo.top + 20); row--) //��ֹrow���
            {
                if (rowInfo[row].leftStatus == EXIST && rowInfo[row + 1].leftStatus == EXIST) //�ж����Ͻǵ��Ƿ����
                {
                    if (
                            (rowInfo[row].leftLine - rowInfo[row - 4].leftLine) > 2 &&
                            (rowInfo[row].leftLine - rowInfo[row - 4].leftLine) < 15 &&
                            (rowInfo[row].leftLine - rowInfo[row + 6].leftLine) > 2 &&
                            (rowInfo[row].leftLine - rowInfo[row + 6].leftLine) < 15
                        )//��ֵ��Ҫ����
                    {
                        imgInfo.PStatus = P_OUT_READY;
                        break;
                    }

                }
            }
        }
    }

    // ���� P_OUT_1 ��
    // �ж��Ƿ񵽴� P_OUT_2 (��p����������)ʱ��
    if (imgInfo.PStatus == P_OUT_READY)
    {
        if(imgInfo.RoadType  == P_L)
        {
            float curv = get_curvature(imgInfo.top + 5, HEIGHT - 3, RIGHT);
#ifdef DEBUG
            printf("curv: %f\n", curv);
#endif
            if(abs(curv) < ConstData.kImageStraightCurvTh)
            {
                imgInfo.PStatus = P_OUT_1;
            }
        }
        else if (imgInfo.RoadType == P_R)
        {
            float curv = get_curvature(imgInfo.top + 5, HEIGHT - 3, LEFT);
#ifdef DEBUG
            printf("curv: %f\n", curv);
#endif
            if(abs(curv) < ConstData.kImageStraightCurvTh)
            {
                imgInfo.PStatus = P_OUT_1;
            }
        }
    }

    if (imgInfo.PStatus == P_OUT_1)
    {
        if(imgInfo.RoadType  == P_L)
        {
            float curv = get_curvature(imgInfo.top + 5, HEIGHT - 3, RIGHT);
#ifdef DEBUG
            printf("curv: %f\n", curv);
#endif
            if(abs(curv) < ConstData.kImageStraightCurvTh)
            {
                imgInfo.PStatus = P_OUT_1;
            }
        }
        else if (imgInfo.RoadType == P_R)
        {
            float curv = get_curvature(imgInfo.top + 5, HEIGHT - 3, LEFT);
#ifdef DEBUG
            printf("curv: %f\n", curv);
#endif
            if(abs(curv) < ConstData.kImageStraightCurvTh)
            {
                imgInfo.PStatus = P_OUT_1;
            }
        }
    }

    // ���� P_OUT_1 ��
    // �ж��Ƿ񵽴� P_OFF (��p����������)ʱ��
    if(imgInfo.PStatus == P_OUT_1) // �����p��״̬, ����� [����p��] ���״̬
    {
        if(imgInfo.RoadType == P_L && (HEIGHT - 10 - (imgInfo.top + 10)) > 0)
        {
            int variance_l = 0; // ����
            GET_VARIANCE(variance_l, left, 10);
            int variance_m = 0; // ����
            GET_VARIANCE(variance_m, mid, 10);

            // ������½��ɰױ������
            static int downside_flag_last_l = 0, downside_check_flag_l = 0; // ���½ǰ�ɫΪ 1, ��ɫΪ 0
            int downside_flag_now_l  =    imageBin[HEIGHT- 1][9] && imageBin[HEIGHT - 1][10]
                && imageBin[HEIGHT - 1][11] && imageBin[HEIGHT - 2][12];

            if(!downside_flag_now_l && downside_flag_last_l && !downside_check_flag_l)
               downside_check_flag_l = 1;

            if (   variance_l <= ConstData.kImagePOutVarianceTh
                && variance_m <= ConstData.kImagePOutVarianceTh
                && downside_check_flag_l)
            {
                imgInfo.PStatus = P_OFF;
                downside_flag_last_l = 0;
                downside_check_flag_l = 0;
            }
            else
            {
                downside_flag_last_l = downside_flag_now_l;
            }
#ifdef DEBUG
            PRINT_LINE_VARIANCE_INFO(variance_l);
            PRINT_LINE_VARIANCE_INFO(variance_m);
            PRINT_FLAG_INFO(downside_check_flag_l);
#endif
        }
        else if (imgInfo.RoadType == P_R && (HEIGHT - 10 - (imgInfo.top + 10)) > 0)
        {
            int variance_r = 0; // ����
            GET_VARIANCE(variance_r, right, 10);
            int variance_m = 0; // ����
            GET_VARIANCE(variance_m, mid, 10);

            // ������½��ɰױ������
            static int downside_flag_last_r = 0, downside_check_flag_r = 0; // ���½ǰ�ɫΪ 1, ��ɫΪ 0
            int downside_flag_now_r  =    imageBin[HEIGHT- 1][WIDTH - 10] && imageBin[HEIGHT - 1][WIDTH - 11]
                                    && imageBin[HEIGHT - 1][WIDTH - 12] && imageBin[HEIGHT - 2][WIDTH - 13];
            if(!downside_flag_now_r && downside_flag_last_r && !downside_check_flag_r)
               downside_check_flag_r = 1;

            if (   variance_r <= ConstData.kImagePOutVarianceTh
                && variance_m <= ConstData.kImagePOutVarianceTh
                && downside_check_flag_r)
            {
                imgInfo.PStatus = P_OFF;
                downside_flag_last_r = 0;
                downside_check_flag_r = 0;
            }
            else
            {
                downside_flag_last_r = downside_flag_now_r;
            }
#ifdef DEBUG
            PRINT_LINE_VARIANCE_INFO(variance_r);
            PRINT_LINE_VARIANCE_INFO(variance_m);
            PRINT_FLAG_INFO(downside_check_flag_r);
#endif
        }

    }
    if(imgInfo.PStatus == P_OFF)
    {
        imgInfo.PStatus = P_NOT_FIND;
        imgInfo.RoadType = Road_None;
    }
}

/**
 * @description: p�����ߺ���
 * @return {*}
 */
void p_repairLine(void)
{
    if (imgInfo.PStatus == P_OUT_1)
    {
        if(imgInfo.RoadType  == P_L)
        {
            float k, b;
            k = (float)(ConstData.kImagePOutRepairLineK * WIDTH) / (imgInfo.top - (HEIGHT - 1));
            b = (float)- k * (HEIGHT - 1);
            add_line(k, b, imgInfo.top, HEIGHT - 1, LEFT);
            recalc_line(imgInfo.top, HEIGHT - 1, LEFT);
        }
        else if (imgInfo.RoadType == P_R)
        {
            float k, b;
            k = (float)(ConstData.kImagePOutRepairLineK * WIDTH) / (HEIGHT - 1 - imgInfo.top);
            b = (float)WIDTH - 1 - k * (HEIGHT - 1);
            add_line(k, b, imgInfo.top, HEIGHT - 1, RIGHT);
            recalc_line(imgInfo.top, HEIGHT - 1, RIGHT);
        }
    }
}

/**
 * @description: �뻷���жϺ���
 * @return {*}
 */
void circle_judge_in(void)
{
    circle_in_flag = 'F';


    PixelTypedef start;

    const int dx[8] = {1, 1, 0,-1,-1,-1, 0, 1,};
    const int dy[8] = {0,-1,-1,-1, 0, 1, 1, 1,};
    /**
     * ���� # ��ΧΪ
     *
     *   3 2 1
     *   4 # 0
     *   5 6 7
     *
     */
    int16_t x, y, tx, ty, s, SearchCompleteFlag = 0, SearchDisruptFlag = 0;
    if (imgInfo.RoadType == Circle_L) // ��ζ�Ż������󻷵�
    {
        int16_t SerialCnt_left = 0;
        //��¼��ʼ��
        start.y = color_TogglePos_left[1].posY;
        start.x = rowInfo[start.y].leftLine;
        //��ʼ������ɨ��
        if(color_TogglePos_left[1].type == WhiteToBlack) s = 5;
        else if(color_TogglePos_left[1].type == BlackToWhite) s = 1;

        x = start.x;
        y = start.y;
        SearchCompleteFlag = 0; // ������ɱ�־λ

        while(1)
        {
            //ǿ�ƽ���,��ֹɨ��ͷ
            int tmpTop = imgInfo.top + 2, s_cnt = 0;
            if( color_toggleCnt_left >= 4 &&
                color_TogglePos_left[4].type == WhiteToBlack &&
                tmpTop < color_TogglePos_left[4].posY - 4)
            {
                tmpTop = color_TogglePos_left[4].posY - 4;
            }
            if(y <= tmpTop || SerialCnt_left >= (HEIGHT<<1)) // С��ɨ����ֹ������һ��ͻ���ȥ��ֵ
            {
                circle_in_flag = 'F';
                break;
            }
            // printf(" $%d %d$ \n", x, y);
            //��ʱ����תɨ��
            for( ; ; ++s, ++s_cnt)
            {
                s = (s + 8) % 8;
                // if(s == 8) s = 0;

                tx = x + dx[s];// ��ΧԪ�ص�����
                ty = y + dy[s];
                if(tx == start.x && ty == start.y)// ɨ����ʼ�㣬�������
                {
                    SearchCompleteFlag = 1;
                    break;
                }
                if(tx == 0 || tx == 1) continue; //��������ǿ�ƿ�����
                if(tx > WIDTH - 1 || ty > HEIGHT - 1 || ty < 0 || s_cnt > 8) // �����߽����ɨ����һȦ
                {
                    SearchDisruptFlag = 1;
                    break;
                }
                if(!imageBin[ty][tx])// ��ɫ�����䣩����x��y
                {
                    x = tx;
                    y = ty;
                    s =  (s - 2 + 8) % 8; // ����һ��ֱ��
                    break;
                }
            }
            if(SearchCompleteFlag == 1)// ��������˳�
            {
                circle_in_flag = 'T';
                break;
            }
            if(SearchDisruptFlag == 1)
            {
                circle_in_flag = 'F';
                break;
            }
            ++SerialCnt_left;// �ǲ���
        }
    }
    else if (imgInfo.RoadType == Circle_R) // ��ζ�Ż������һ���
    {
        int16_t SerialCnt_right = 0;
        //��¼��ʼ��
        start.y = color_TogglePos_right[1].posY;
        start.x = rowInfo[start.y].rightLine;
        //��ʼ������ɨ��
        if(color_TogglePos_right[1].type == WhiteToBlack) s = 5;
        else if(color_TogglePos_right[1].type == BlackToWhite) s = 3;

        x = start.x;
        y = start.y;
        SearchCompleteFlag = 0; // ������ɱ�־λ

        while(1)
        {
            //ǿ�ƽ���,��ֹɨ��ͷ
            int tmpTop = imgInfo.top + 2, s_cnt = 0;
            if( color_toggleCnt_right >= 4 &&
                color_TogglePos_right[4].type == WhiteToBlack &&
                tmpTop < color_TogglePos_right[4].posY - 4)
            {
                tmpTop = color_TogglePos_right[4].posY - 4;
            }
            if(y <= tmpTop || SerialCnt_right >= (HEIGHT<<1)) // С��ɨ����ֹ������һ��ͻ���ȥ��ֵ
            {
                circle_in_flag = 'F';
                break;
            }
            // printf(" $%d %d$ \n", x, y);
            //��ʱ����תɨ��
            for( ; ; ++s, ++s_cnt)
            {
                s = (s + 8) % 8;
                // if(s == 8) s = 0;

                tx = x + dx[s];// ��ΧԪ�ص�����
                ty = y + dy[s];
                if(tx == start.x && ty == start.y)// ɨ����ʼ�㣬�������
                {
                    SearchCompleteFlag = 1;
                    break;
                }
                if(tx == WIDTH - 1 || tx == WIDTH - 2 ) continue; //��������ǿ�ƿ�����
                if(tx < 0 || ty > HEIGHT - 1 || ty < 0 || s_cnt > 8)
                {
                    SearchDisruptFlag = 1;
                    break;
                }
                if(!imageBin[ty][tx])// ��ɫ�����䣩����x��y
                {
                    x = tx;
                    y = ty;
                    s =  (s - 2 + 8) % 8; // ����һ��ֱ��
                    break;
                }
            }
            if(SearchCompleteFlag == 1)// ��������˳�
            {
                circle_in_flag = 'T';
                break;
            }
            if(SearchDisruptFlag == 1)
            {
                circle_in_flag = 'F';
                break;
            }
            ++SerialCnt_right;// �ǲ���

        }
    }
}

int up_point = MISS;
const int circle_rad = 8;
const int circle_getPoint_del = 5;
float circle_k = 1.02;




/**
 * @description: ������⺯��
 * @return {*}
 */
void circle_detect(void)
{
    if (isCircle_flag_1 == 'T' && isCircle_flag_2 == 'T' && imgInfo.CircleStatus == CIRCLE_NOT_FIND ) // �����о�ΪT �һ���δ�ҵ� ��˵���ҵ�����
    {

        if (color_toggleCnt_left - color_toggleCnt_right >= 3 && imgInfo.RoadType != Circle_R) // ͨ�������� �ж��󻷵������һ��� ��ཻ�����϶� ��Ϊ�󻷵�
        {
            // ��ǿ����, Ҫ����������ٵ�һ������һ������������Χ�ڵ�����С����ֵ
            // ȡ��һ������(BlackToWhite)�ĵ���Ϊ��ʼ��, ȡ����������(BlackToWhite)�ĵ���Ϊ�յ�
            int variance_r = 0;
            GET_VARIANCE(variance_r, right, 6); // ���㷽��
#ifdef DEBUG
            printf("Circle variance_r: %d\n", variance_r);
#endif
            if (variance_r < ConstData.kImageLineVarianceTh && imgInfo.top < bigCircleTop)
            {
                imgInfo.CircleStatus = CIRCLE_FIND; // ��� [���ֻ���] ״̬
                imgInfo.RoadType = Circle_L;
            }
        }
        else if (color_toggleCnt_left - color_toggleCnt_right <= -3 && imgInfo.RoadType != Circle_L)
        {
            // ͬ��
            int variance_l = 0;
            GET_VARIANCE(variance_l, left, 6); // ���㷽��
#ifdef DEBUG
            printf("Circle variance_l: %d\n", variance_l);
#endif
            if (variance_l < ConstData.kImageLineVarianceTh && imgInfo.top < bigCircleTop)
            {
                imgInfo.CircleStatus = CIRCLE_FIND; // ��� [���ֻ���] ״̬
                imgInfo.RoadType = Circle_R;
            }
        }
    }

    if (imgInfo.CircleStatus == CIRCLE_FIND) // ������ֻ���, ����� [�뻷��] ���״̬
    {
        circle_judge_in(); // �ж��Ƿ��뻷������
#ifdef DEBUG
        printf("circle_in_flag = %c\n", circle_in_flag);
#endif

        if (imgInfo.RoadType == Circle_L)
        {
            static int leftdown_flag_last_F2I = 0, leftdown_check_flag_F2I = 0; // ���½ǰ�ɫΪ 1, ��ɫΪ 0
            // ������½��ɰױ������
            int leftdown_flag_now = imageBin[HEIGHT- 1][1] && imageBin[HEIGHT - 1][2]
                && imageBin[HEIGHT - 1][3] && imageBin[HEIGHT - 2][3] && PIXEL(HEIGHT - 1, left);

            if(!leftdown_flag_now && leftdown_flag_last_F2I && !leftdown_check_flag_F2I)
                leftdown_check_flag_F2I = 1;

            if (circle_in_flag == 'F' && leftdown_check_flag_F2I) // ���û���ҵ���ɫ�ջ�, ��˵��Ӧ���뻷��, ���״̬λ
            {
#if defined (__ON_ROBOT__)
                start_integrating_angle();
#endif
                imgInfo.CircleStatus = CIRCLE_IN;
                leftdown_flag_last_F2I = 0;
                leftdown_check_flag_F2I = 0;
            }
            else
            {
                leftdown_flag_last_F2I = leftdown_flag_now;
            }
        }
        else if (imgInfo.RoadType == Circle_R)
        {
            static int rightdown_flag_last_F2I = 0, rightdown_check_flag_F2I = 0; // ���½ǰ�ɫΪ 1, ��ɫΪ 0
            // ������½��ɰױ������
            int rightdown_flag_now = imageBin[HEIGHT- 1][WIDTH - 2] && imageBin[HEIGHT - 1][WIDTH - 3]
                                    && imageBin[HEIGHT - 1][WIDTH - 4] && imageBin[HEIGHT - 2][WIDTH - 4]
                                    && PIXEL(HEIGHT - 1, right);
            if(!rightdown_flag_now && rightdown_flag_last_F2I && !rightdown_check_flag_F2I)
               rightdown_check_flag_F2I = 1;

            if (circle_in_flag == 'F' && rightdown_check_flag_F2I) // ���û���ҵ���ɫ�ջ�, ��˵��Ӧ���뻷��, ���״̬λ
            {
#if defined (__ON_ROBOT__)
                start_integrating_angle();
#endif
                imgInfo.CircleStatus = CIRCLE_IN;
                rightdown_flag_last_F2I = 0;
                rightdown_check_flag_F2I = 0;
            }
            else
            {
                rightdown_flag_last_F2I = rightdown_flag_now;
            }
        }

    }

    // if (imgInfo.CircleStatus == CIRCLE_IN) // ����뻷��״̬, ����� [��������] ���״̬
    // {
    //     if(imgInfo.RoadType == Circle_L)
    //     {
    //         static int rightdown_flag_last_I2P = 0, rightdown_check_flag_I2P = 0; // ���½ǰ�ɫΪ 1, ��ɫΪ 0
    //         // ������½��ɰױ������
    //         int rightdown_flag_now = imageBin[HEIGHT- 1][WIDTH - 2] && imageBin[HEIGHT - 1][WIDTH - 3]
    //                                 && imageBin[HEIGHT - 1][WIDTH - 4] && imageBin[HEIGHT - 2][WIDTH - 4]
    //                                 && PIXEL(HEIGHT - 1, right);
    //         if(!rightdown_flag_now && rightdown_flag_last_I2P && !rightdown_check_flag_I2P)
    //            rightdown_check_flag_I2P = 1;

    //         if (rightdown_check_flag_I2P)
    //         {
    //             imgInfo.CircleStatus = CIRCLE_PASSING;
    //             rightdown_flag_last_I2P = 0;
    //             rightdown_check_flag_I2P = 0;
    //         }
    //         else
    //         {
    //             rightdown_flag_last_I2P = rightdown_flag_now;
    //         }
    //     }
    //     else if(imgInfo.RoadType == Circle_R)
    //     {
    //         static int leftdown_flag_last_I2P = 0, leftdown_check_flag_I2P = 0; // ���½ǰ�ɫΪ 1, ��ɫΪ 0
    //         // ������½��ɰױ������
    //         int leftdown_flag_now = imageBin[HEIGHT- 1][1] && imageBin[HEIGHT - 1][2]
    //             && imageBin[HEIGHT - 1][3] && imageBin[HEIGHT - 2][3] && PIXEL(HEIGHT - 1, left);

    //         if(!leftdown_flag_now && leftdown_flag_last_I2P && !leftdown_check_flag_I2P)
    //             leftdown_check_flag_I2P = 1;

    //         if (leftdown_check_flag_I2P)
    //         {
    //             imgInfo.CircleStatus = CIRCLE_PASSING;
    //             leftdown_flag_last_I2P = 0;
    //             leftdown_check_flag_I2P = 0;
    //         }
    //         else
    //         {
    //             leftdown_flag_last_I2P = leftdown_flag_now;
    //         }
    //     }
    // }
    if (imgInfo.CircleStatus == CIRCLE_IN) // ����뻷��״̬, ����� [��������] ���״̬
    {
        if  (
                color_toggleCnt_left <= 1 && color_toggleCnt_right <= 1 // ����������ཻ������С�ڵ���1, ��˵�����ھ�������
                &&
#if defined (__ON_ROBOT__)
                ((imgInfo.RoadType == Circle_L && check_yaw_angle() >= DEGREE_45)
                 ||
                 (imgInfo.RoadType == Circle_R && check_yaw_angle() <= -DEGREE_45))
                &&
#endif
                ((! imageBin[HEIGHT - 1][WIDTH - 1] && ! imageBin[HEIGHT - 1][WIDTH - 2] && ! imageBin[HEIGHT - 1][WIDTH - 3])
                 ||
                (! imageBin[HEIGHT - 1][0] && ! imageBin[HEIGHT - 1][1] && !imageBin[HEIGHT - 1][2]))
                // ����������ཻ������С�ڵ���1, ��˵�����ھ�������
            )
        {
            imgInfo.CircleStatus = CIRCLE_PASSING;
#if defined (__ON_ROBOT__)
            stop_interating_angle();
#endif
        }
    }
    if(imgInfo.CircleStatus == CIRCLE_PASSING) // �����������״̬, ����� [������] ���״̬
    {
        if(imgInfo.RoadType  == Circle_L)
        {

            // �˷���ΪѰ�ҽǵ�, ���Ϸ����·��ĵ������Ҳ�
            for (int row = HEIGHT - 10; row > (imgInfo.top + 20); row--) //��ֹrow���
            {
                if (rowInfo[row].rightStatus == EXIST && rowInfo[row + 1].rightStatus == EXIST) //�ж����Ͻǵ��Ƿ����
                {
                    if (
                               (rowInfo[row - 4].rightLine - rowInfo[row].rightLine) > 2
                            && (rowInfo[row - 4].rightLine - rowInfo[row].rightLine) < 15
                            && (rowInfo[row + 6].rightLine - rowInfo[row].rightLine) > 2
                            && (rowInfo[row + 6].rightLine - rowInfo[row].rightLine) < 15
                    )//��ֵ��Ҫ����
                    {
                        imgInfo.CircleStatus = CIRCLE_OUT;
#if defined (__ON_ROBOT__)
                        start_integrating_angle();
#endif
                        break;
                    }

                }
            }
        }
        else if (imgInfo.RoadType  == Circle_R)
        {
            for (int row = HEIGHT - 10; row > (imgInfo.top + 20); row--) //��ֹrow���
            {
                if (rowInfo[row].leftStatus == EXIST && rowInfo[row + 1].leftStatus == EXIST) //�ж����Ͻǵ��Ƿ����
                {
                    if (
                            (rowInfo[row].leftLine - rowInfo[row - 4].leftLine) > 2 &&
                            (rowInfo[row].leftLine - rowInfo[row - 4].leftLine) < 15 &&
                            (rowInfo[row].leftLine - rowInfo[row + 6].leftLine) > 2 &&
                            (rowInfo[row].leftLine - rowInfo[row + 6].leftLine) < 15
                        )//��ֵ��Ҫ����
                    {
                        imgInfo.CircleStatus = CIRCLE_OUT;
#if defined (__ON_ROBOT__)
                        start_integrating_angle();
#endif
                        break;
                    }

                }
            }
        }
    }

    if(imgInfo.CircleStatus == CIRCLE_OUT) // ���������״̬, ����� [��������] ���״̬
    {
        if(imgInfo.RoadType == Circle_L && (HEIGHT - 10 - (imgInfo.top + 10)) > 0)
        {
            int variance_r = 0; // ����
            GET_VARIANCE(variance_r, right, 10);
            int variance_m = 0; // ����
            GET_VARIANCE(variance_m, mid, 10);

            // ������½��ɰױ������
            static int downside_flag_last_r = 0, downside_check_flag_r = 0; // ���½ǰ�ɫΪ 1, ��ɫΪ 0
            int downside_flag_now_r  =    imageBin[HEIGHT- 1][WIDTH - 2] && imageBin[HEIGHT - 1][WIDTH - 3]
                                    && imageBin[HEIGHT - 1][WIDTH - 4] && imageBin[HEIGHT - 2][WIDTH - 4];
            if(!downside_flag_now_r && downside_flag_last_r && !downside_check_flag_r)
               downside_check_flag_r = 1;
#ifdef DEBUG
            PRINT_LINE_VARIANCE_INFO(variance_r);
            PRINT_LINE_VARIANCE_INFO(variance_m);
            printf("downside_flag_last_r = %d\n", downside_flag_last_r);
            printf("downside_flag_now_r = %d\n", downside_flag_now_r);
            printf("downside_check_flag_r = %d\n", downside_check_flag_r);
#endif

            if (
                (variance_r <= ConstData.kImageLineVarianceTh
                && variance_m <= ConstData.kImageLineVarianceTh
                && downside_check_flag_r)
#if defined (__ON_ROBOT__)
                || check_yaw_angle() >= DEGREE_80
#endif
                )
            {
                imgInfo.CircleStatus = CIRCLE_OFF;
#if defined (__ON_ROBOT__)
                stop_interating_angle();
#endif
                downside_flag_last_r = 0;
                downside_check_flag_r = 0;
            }
            else
            {
                downside_flag_last_r = downside_flag_now_r;
            }
        }
        else if (imgInfo.RoadType == Circle_R && (HEIGHT - 10 - (imgInfo.top + 10)) > 0)
        {
            int variance_l = 0; // ����
            GET_VARIANCE(variance_l, left, 10);
            int variance_m = 0; // ����
            GET_VARIANCE(variance_m, mid, 10);

            // ������½��ɰױ������
            static int downside_flag_last_l = 0, downside_check_flag_l = 0; // ���½ǰ�ɫΪ 1, ��ɫΪ 0
            int downside_flag_now_l  =    imageBin[HEIGHT- 1][1] && imageBin[HEIGHT - 1][2]
                && imageBin[HEIGHT - 1][3] && imageBin[HEIGHT - 2][3];

            if(!downside_flag_now_l && downside_flag_last_l && !downside_check_flag_l)
               downside_check_flag_l = 1;
#ifdef DEBUG
            PRINT_LINE_VARIANCE_INFO(variance_l);
            PRINT_LINE_VARIANCE_INFO(variance_m);
            printf("downside_flag_last_l = %d\n", downside_flag_last_l);
            printf("downside_flag_now_l = %d\n", downside_flag_now_l);
            printf("downside_check_flag_l = %d\n", downside_check_flag_l);
#endif
            if (
                (variance_l <= ConstData.kImageLineVarianceTh
                && variance_m <= ConstData.kImageLineVarianceTh
                && downside_check_flag_l)
#if defined (__ON_ROBOT__)
                || check_yaw_angle() <= -DEGREE_80
#endif
                )
            {
                imgInfo.CircleStatus = CIRCLE_OFF;
#if defined (__ON_ROBOT__)
                stop_interating_angle();
#endif
                downside_flag_last_l = 0;
                downside_check_flag_l = 0;
            }
            else
            {
                downside_flag_last_l = downside_flag_now_l;
            }
        }
    }

    if(imgInfo.CircleStatus == CIRCLE_OFF) // �����������״̬, ����� [��������] ���״̬
    {
            if(
                imgInfo.RoadType == Circle_L
              )
            {
                // ������½��ɰױ������
                static int downside_flag_last_l = 0, downside_check_flag_l = 0; // ���½ǰ�ɫΪ 1, ��ɫΪ 0
                int downside_flag_now_l  =    imageBin[HEIGHT- 1][1] && imageBin[HEIGHT - 1][2]
                    && imageBin[HEIGHT - 1][3] && imageBin[HEIGHT - 2][3];

                if(!downside_flag_now_l && downside_flag_last_l && !downside_check_flag_l)
                    downside_check_flag_l = 1;

                if (downside_check_flag_l)// && abs(color_toggleCnt_left - color_toggleCnt_right) <= 1)
                {
                    imgInfo.CircleStatus = CIRCLE_NOT_FIND;
                    imgInfo.RoadType = Road_None;
                    downside_flag_last_l = 0;
                    downside_check_flag_l = 0;
                }
                else
                {
                    downside_flag_last_l = downside_flag_now_l;
                }
            }
            else if(
                imgInfo.RoadType == Circle_R
                )
            {
                // ������½��ɰױ������
                static int downside_flag_last_r = 0, downside_check_flag_r = 0; // ���½ǰ�ɫΪ 1, ��ɫΪ 0
                int downside_flag_now_r  =    imageBin[HEIGHT- 1][WIDTH - 2] && imageBin[HEIGHT - 1][WIDTH - 3]
                                        && imageBin[HEIGHT - 1][WIDTH - 4] && imageBin[HEIGHT - 2][WIDTH - 4];
                if(!downside_flag_now_r && downside_flag_last_r && !downside_check_flag_r)
                    downside_check_flag_r = 1;
#ifdef DEBUG
                printf("Circle_R OFF -> Road_None: \n");
                printf("    downside_flag_last_r  = %c\n", downside_flag_last_r == 1?'W':'B');
                printf("    downside_flag_now_r   = %c\n", downside_flag_now_r == 1?'W':'B');
                printf("    downside_check_flag_r = %c\n", downside_check_flag_r == 1?'T':'F');
#endif
                if (downside_check_flag_r)// && abs(color_toggleCnt_left - color_toggleCnt_right) <= 1)
                {
                    imgInfo.CircleStatus = CIRCLE_NOT_FIND;
                    imgInfo.RoadType = Road_None;
                    downside_flag_last_r = 0;
                    downside_check_flag_r = 0;
                }
                else
                {
                    downside_flag_last_r = downside_flag_now_r;
                }
            }
        }
}


/**
 * @description: �������ߺ���
 * @return {*}
 */
void circle_repairLine(void)
{
    // �뻷����
    if (imgInfo.CircleStatus == CIRCLE_IN) // ��ʱ�����뻷״̬
    {
        if (imgInfo.RoadType == Circle_L) // ��Բ��
        {
            float k, b;
            k = (ConstData.kImageCircleInRepairLineK * WIDTH) / (HEIGHT - 1 - imgInfo.top);
            b = WIDTH - 1 - k * (HEIGHT - 1);
            add_line(k, b, imgInfo.top, HEIGHT - 1, RIGHT);
            recalc_line(imgInfo.top, HEIGHT - 1, RIGHT);
        }
        if (imgInfo.RoadType == Circle_R) //��Բ��
        {
            float k, b;
            k = (ConstData.kImageCircleInRepairLineK * WIDTH) / (imgInfo.top - (HEIGHT - 1));
            b = - k * (HEIGHT - 1);
            add_line(k, b, imgInfo.top, HEIGHT - 1, LEFT);
            recalc_line(imgInfo.top, HEIGHT - 1, LEFT);
        }
    }
    // if (imgInfo.CircleStatus == CIRCLE_PASSING)
    // {
    //     if (imgInfo.RoadType == Circle_L) // �󻷵����油������ƫ��, �����Ʒ�Χ
    //     {
    //         for (int row = imgInfo.top + 1; row < HEIGHT; ++row)
    //         {
    //             // int tmp = rowInfo[row].leftLine + regWidth[row] / 2;
    //             if (rowInfo[row].rightLine > rowInfo[row].leftLine + regWidth[row])
    //                 rowInfo[row].rightLine = rowInfo[row].leftLine + regWidth[row];
    //             rowInfo[row].rightLine = rowInfo[row].rightLine - (ConstData.kImagePassingOffset << 1);
    //             rowInfo[row].midLine = rowInfo[row].midLine - ConstData.kImagePassingOffset;
    //             if (rowInfo[row].midLine < 1)
    //                 rowInfo[row].midLine = 1;
    //         }
    //     }
    //     else if (imgInfo.RoadType == Circle_R) // �һ������油������ƫ��, �����Ʒ�Χ
    //     {
    //         for (int row = imgInfo.top + 1; row < HEIGHT; ++row)
    //         {
    //             // int tmp = rowInfo[row].rightLine - regWidth[row] / 2; // rowInfo[row].midLine < tmp ? tmp - 5:
    //             if (rowInfo[row].leftLine < rowInfo[row].rightLine - regWidth[row])
    //                 rowInfo[row].leftLine = rowInfo[row].rightLine - regWidth[row];
    //             rowInfo[row].leftLine = rowInfo[row].leftLine + (ConstData.kImagePassingOffset << 1);
    //             rowInfo[row].midLine = rowInfo[row].midLine + ConstData.kImagePassingOffset;
    //             if (rowInfo[row].midLine > WIDTH - 2)
    //                 rowInfo[row].midLine = WIDTH - 2;
    //         }
    //     }
    // }
    //��������
    if (imgInfo.CircleStatus == CIRCLE_OUT) //��ʱ���ڳ���״̬
    {
        //��Բ��
        float Det_R = 0.0;
        if (imgInfo.RoadType == Circle_L)
        {
            Det_R = ConstData.kImageCircleOutRepairLineK * (WIDTH - 1) / (HEIGHT - 5 - (imgInfo.top + 1));
            for (int y = HEIGHT - 5; y > imgInfo.top; y--)
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

        float Det_L= 0.0;

        if (imgInfo.RoadType == Circle_R)
        {
            Det_L = ConstData.kImageCircleOutRepairLineK * (WIDTH - 1) / (HEIGHT - 5 - imgInfo.top);
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


void barnOut_repairLine(void)
{
    for (int row = HEIGHT - 1; row > imgInfo.top + 2; --row)
    {
        float k, b;
        k = (ConstData.kImageBarnOutRepairLineK * WIDTH) / (HEIGHT - 1 - imgInfo.top);
        b = WIDTH - 1 - k * (HEIGHT - 1);
        add_line(k, b, imgInfo.top, HEIGHT - 1, RIGHT);
        recalc_line(imgInfo.top, HEIGHT - 1, RIGHT);
    }
}

uint8_t stop_detect(void)
{
    int blackCount = 0;

    for (int i = 0; i <= WIDTH - 1; ++i)
    {
        if (!imageBin[HEIGHT - 1][i])
            ++blackCount;
    }
    return blackCount > WIDTH * 0.8;
}


const int weight_base[60] = {                        //0Ϊͼ�����
    0 , 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1 , 1, 3, 3, 5, 5,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10, 5, 5, 5, 5, 5,
    3 , 3, 3, 1, 1, 1, 0, 0, 0, 0,
};    //����    //ע��б�ʱ仯���������,Ҫƽ��
const int weight_circle[60] = {                        //0Ϊͼ�����
    0 , 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1 , 1, 3, 3, 5, 5,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10, 5, 5, 5, 5, 5,
    3 , 3, 3, 1, 1, 1, 0, 0, 0, 0,
};    //����    //ע��б�ʱ仯���������,Ҫƽ��
const int weight_p[60] = {                        //0Ϊͼ�����
    0 , 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1 , 1, 3, 3, 5, 5,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10, 5, 5, 5, 5, 5,
    3 , 3, 3, 1, 1, 1, 0, 0, 0, 0,
};    //����    //ע��б�ʱ仯���������,Ҫƽ��
const int weight_fork[60] = {                        //0Ϊͼ�����
    0 , 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1 , 1, 3, 3, 5, 5,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10,10,10,10,10,10,
    10,10,10,10,10, 5, 5, 5, 5, 5,
    3 , 3, 3, 1, 1, 1, 0, 0, 0, 0,
};    //����    //ע��б�ʱ仯���������,Ҫƽ��
/**
 * @description: ��ȡ��ǰλ�ú�����λ�õ�ƫ����ݸ�PID����
 * @param {*}
 * @return {*}
 */
void get_error(void)//TODO ʵ�ֶ�̬����ǰհ
{
    int avePos = 0, predictedPass = 0;
    float spd = 3.4;// = get_speed() ֮��д����speed��̬ѡ��ǰհ��ģʽ
    predictedPass = max (imgInfo.bottom - (int)(spd * 10), imgInfo.top + 3); //֮�����ʵ�����д����ϵʽ
    avePos = (rowInfo[predictedPass].midLine + rowInfo[predictedPass + 1].midLine + rowInfo[predictedPass + 2].midLine) / 3;
    /*  90  -->  HEIGHT - 25    25 / 90 = 0.28
        120 -->  HEIGHT - 34




    */
    imgInfo.error = avePos-(WIDTH/2);
    // printf("%d :   %d\n", predictedPass, rowInfo[predictedPass].midLine);
    // printf("avePos = %d\nimgInfo.error = %d\n", avePos, imgInfo.error);

    // int avePos, predictedPass;
    // float spd = 2.5;// = get_speed() ֮��д����speed��̬ѡ��ǰհ��ģʽ
    // predictedPass = min(spd * 2, imgInfo.top - 4); //֮�����ʵ�����д����ϵʽ
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


void calc_globalError(void)
{
    // for(i = startRow; i > validRow + 1; i --) {
    // weightSum += weight1[i];
    // lineSum += weight1[i] * middleLine[i];
    // }   //��������ɨ

    // angleErr = (float)lineSum / weightSum - middleStandard;
    // midline_fff = midline_ff;
    // midline_ff  = midline_f;
    // midline_f = angleErr;
    // angleErr = midline_fff * 0.50f + midline_ff * 0.30f + midline_f * 0.20f;
}


/**
 * @description: ��ȡָ������������߷����
 * @param {uint8_t} select_top
 * @param {uint8_t} select_bottom
 * @param {LineTypeEnum} type
 * @return {int} var �õ��ķ���
 */
int get_variance(uint8_t select_top, uint8_t select_bottom, LineTypeEnum type)
{
    int var = 0;
    float pred_k = 0.0, pred_b = 0.0;
    switch (type)
    {
        case LEFT:
            pred_k = (float)(rowInfo[select_bottom].leftLine - rowInfo[select_top].leftLine) / (select_bottom - select_top);
            pred_b = (float)rowInfo[(int)((select_top + select_bottom) / 2)].leftLine - pred_k * ((select_top + select_bottom) / 2.0);
            for (int row = select_bottom; row > select_top; --row)
            {
                var +=  (rowInfo[row].leftLine - (int)(pred_k * row + pred_b))
                    * (rowInfo[row].leftLine - (int)(pred_k * row + pred_b));
            }
            break;
        case RIGHT:
            pred_k = (float)(rowInfo[select_bottom].rightLine - rowInfo[select_top].rightLine) / (select_bottom - select_top);
            pred_b = (float)rowInfo[(int)((select_top + select_bottom) / 2)].rightLine - pred_k * ((select_top + select_bottom) / 2.0);
            for (int row = select_bottom; row > select_top; --row)
            {
                var +=  (rowInfo[row].rightLine - (int)(pred_k * row + pred_b))
                    * (rowInfo[row].rightLine - (int)(pred_k * row + pred_b));
            }
            break;

        case MID:
            break;
        default:
            break;

    }

    return var == 0 ? 0x3f3f3f3f : var;
}


uint8_t judge_lineBeginLost(LineTypeEnum type)
{
    uint8_t ans = 0;
    switch (type)
    {
        case LEFT:
            ans = imageBin[HEIGHT - 1][0] && imageBin[HEIGHT - 2][0] && imageBin[HEIGHT - 4][0] && imageBin[HEIGHT - 8][0]
                    && imageBin[HEIGHT - 1][4] && imageBin[HEIGHT - 2][2];
                    // && rowInfo[HEIGHT - 1].leftStatus == LOST && rowInfo [HEIGHT - 2].leftStatus == LOST
                    // && rowInfo [HEIGHT - 4].leftStatus == LOST && rowInfo [HEIGHT - 8].leftStatus == LOST;
            break;
        case RIGHT:
            ans = imageBin[HEIGHT - 1][WIDTH - 1] && imageBin[HEIGHT - 2][WIDTH - 1] && imageBin[HEIGHT - 4][WIDTH - 1] && imageBin[HEIGHT - 8][WIDTH - 1]
                    && imageBin[HEIGHT - 1][WIDTH - 5] && imageBin[HEIGHT - 2][WIDTH - 3];
                    // && rowInfo[HEIGHT - 1].rightStatus == LOST && rowInfo [HEIGHT - 2].rightStatus == LOST
                    // && rowInfo [HEIGHT - 4].rightStatus == LOST && rowInfo [HEIGHT - 8].rightStatus == LOST;
            break;
        default:
            break;
    }
    return ans == 1? 'T' : 'F';
}


/**
 * @description: �ж���/��/���ߵ�������
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
            for (int j = select_top; j < select_bottom; ++j)//�ӵ�10�п�ʼ����ֹ��������ϵ���Ӱ���ж�
            {
                delta = rowInfo[j + 1].leftLine - rowInfo[j].leftLine;       //left�ߵ�ƫ��
                if (delta >= TH_ContinuityDelta || delta <= -TH_ContinuityDelta)
                {
                    return j;
                }
            }
            return 0;

            break;
        case MID:
            for (int j = select_top; j < select_bottom; ++j)//�ӵ�10�п�ʼ����ֹ��������ϵ���Ӱ���ж�
            {
                delta = rowInfo[j + 1].midLine - rowInfo[j].midLine;       //left�ߵ�ƫ��
                if (delta >= TH_ContinuityDelta || delta <= -TH_ContinuityDelta)
                {
                    return j;
                }
            }
            return 0;

            break;
        case RIGHT:
            for (int j = select_top; j < select_bottom; ++j)//�ӵ�10�п�ʼ����ֹ��������ϵ���Ӱ���ж�
            {
                delta = rowInfo[j + 1].rightLine - rowInfo[j].rightLine;       //left�ߵ�ƫ��
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
 * @description: �����������
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
    //����ķ��ű�ʾ����
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
 * @description: ���ߺ����¼�������
 * @param {uint8_t} select_top
 * @param {uint8_t} select_bottom
 * @param {LineTypeEnum} type
 * @return {*}
 */
void recalc_line(uint8_t select_top, uint8_t select_bottom, LineTypeEnum type)
{
    for(int y = select_top + 1; y <= select_bottom; ++ y)
    {
        if(rowInfo[y].leftLine <= rowInfo[y].rightLine)
        {
            rowInfo[y].leftStatus = EXIST;
            rowInfo[y].rightStatus = EXIST;
            rowInfo[y].midLine = (rowInfo[y].leftLine + rowInfo[y].rightLine) / 2;
            rowInfo[y].width = rowInfo[y].rightLine - rowInfo[y].leftLine;
        }

    }
}


/**
 * @description: ���ߣ���Χ���ұգ�
 * @param {float} k ����б�� ͨ��least_squares()������ȡ
 * @param {float} b ����ƫ�� ͨ��least_squares()������ȡ
 * @param {uint8_t} select_top ��Χ��㣨���������ڣ�
 * @param {uint8_t} select_bottom ��Χ�յ㣨�������ڣ�
 * @param {LineTypeEnum} type ���ߵĶ������� {LEFT/MID/RIGHT}
 * @return {*}
 */
void add_line(float k, float b, uint8_t select_top, uint8_t select_bottom, LineTypeEnum type)
{
    int tmp = 0;
    switch (type)
    {
        case LEFT:
        {
            for(int i = select_top + 1; i <= select_bottom; ++i)
            {
                tmp = k * i + b;
                if(tmp < 0) tmp = 0;
                if(tmp > WIDTH - 1) tmp = WIDTH - 1;
                if(tmp > rowInfo[i].leftLine)
                    rowInfo[i].leftLine = tmp;
                if(rowInfo[i].leftLine < 0)
                    rowInfo[i].leftLine = 0;
            }
            break;
        }
        case MID:
        {
            for(int i = select_top + 1; i <= select_bottom; ++i)
            {
                rowInfo[i].midLine = k * i + b;
            }
            break;
        }
        case RIGHT:
        {
            for(int i = select_top + 1; i <= select_bottom; ++i)
            {
                tmp = k * i + b;
                if(tmp < 0) tmp = 0;
                if(tmp > WIDTH - 1) tmp = WIDTH - 1;
                if(tmp < rowInfo[i].rightLine)
                    rowInfo[i].rightLine = tmp;
                if(rowInfo[i].rightLine > WIDTH - 1)
                    rowInfo[i].rightLine = WIDTH - 1;
            }
            break;
        }
        default:
            break;
    }
}

/**
 * @description: ��С���˷����ֱ��
 *               ע�⣺�����һ��ͼ�������ϵ��һ���������Ͻ�Ϊԭ�㣬����Ϊx_t������Ϊy_t��
 *               �� y_t = $k$ * x_t + $b$ ������ԭ����ϵֱ�Ӵ��뵱ǰͼ��߶� h_0 ���������Ӧ�� ������ w_0
 * @param {float *} k ����ֱ��б��
 * @param {float *} b ����ֱ��ƫ��
 * @param {uint8_t} select_top ѡ����Χ�Ŀ�ʼ���߶ȷ�Χ��
 * @param {uint8_t} select_bottom ѡ����Χ�Ľ������߶ȷ�Χ��
 * @param {LeastSquaresObjectEnum} type ��Ҫ��ϵ�ֱ�����ͣ���߽�/�ұ߽�/���ߣ�
 * @return {*} �޷���ֵ��ͨ������ $k$ �� $b$ �õ�ֱ�߲���
 */
void least_squares(float * k, float * b, uint8_t select_top, uint8_t select_bottom, LineTypeEnum type)
{
    int sumx = 0;                      /* sum of x     */
    int sumx2 = 0;                     /* sum of x**2  */
    int sumxy = 0;                     /* sum of x * y */
    int sumy = 0;                      /* sum of y     */
    int sumy2 = 0;                     /* sum of y**2  */
    int n = select_bottom - select_top + 1;

    switch(type) //��ͳ��С���˷��޷���ϴ�ֱ�ģ�����Խ����Խ��������������Ĵ�ֱ���Ӧ�ñȽ϶࣬����������תһ��������
    {
        case LEFT:
            for (int i = select_top; i <= select_bottom; ++i)
            {
                sumx  += i; //i ��Ϊx��ֵ
                sumx2 += i * i;
                sumxy += i * rowInfo[i].leftLine;
                sumy  += rowInfo[i].leftLine;
                sumy2 += rowInfo[i].leftLine * rowInfo[i].leftLine;
            }
            break;
        case MID:
            for (int i = select_top; i <= select_bottom; ++i)
            {
                sumx  += i; //i ��Ϊx��ֵ
                sumx2 += i * i;
                sumxy += i * rowInfo[i].midLine;
                sumy  += rowInfo[i].midLine;
                sumy2 += rowInfo[i].midLine * rowInfo[i].midLine;
            }
            break;
        case RIGHT:
            for (int i = select_top; i <= select_bottom; ++i)
            {
                sumx  += i; //i ��Ϊx��ֵ
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
        *k = 180;//�����Ƚϴ��������
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
 * @description: ���η�Χ�������ֱ�ߣ�����״̬��
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
    if (type == 0)  //�������
    {
        /**����sumX sumY**/
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
        averageX = (float)sumX / (sumlines1 + sumlines2);     //x��ƽ��ֵ
        averageY = (float)sumY / (sumlines1 + sumlines2);     //y��ƽ��ֵ
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
    else if (type == 1)     //�������
    {
        /**����sumX sumY**/
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
        averageX = (float)sumX / (sumlines1 + sumlines2);     //x��ƽ��ֵ
        averageY = (float)sumY / (sumlines1 + sumlines2);     //y��ƽ��ֵ
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
    else if (type == 2)         //�������
    {
        /**����sumX sumY**/
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
        averageX = (float)sumX / (sumlines1 + sumlines2);     //x��ƽ��ֵ
        averageY = (float)sumY / (sumlines1 + sumlines2);     //y��ƽ��ֵ
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

