/*
 * @Author: ROY1994
 * @Date: 2022-02-04 14:01:30
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-03-05 11:40:18
 * @FilePath: \myImageDeal_v0.1\ImageDeal.cpp
 * @Description: ���ߣ�Ԫ���жϵ���Ҫ������
 */
#include "ImageDeal.h"

//#define DEBUG

extern uint8 mt9v03x_image[HEIGHT][WIDTH];//ԭ�Ҷ�ͼ
extern uint8 imageBin[HEIGHT][WIDTH];//��ֵ��ͼ��

pixel leftLineSerial[HEIGHT<<1],rightLineSerial[HEIGHT<<1];//�����������������
int16 leftLineTmpCnt,rightLineTmpCnt;//�����������������

int16 leftLine[HEIGHT] = {0}, rightLine[HEIGHT] = {WIDTH - 1}, midLine[HEIGHT] = {0}, width[HEIGHT] = {0};//ÿ���߶ȵ���/��/���߼����

int16 regWidth[HEIGHT] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,23,29,36,39,51,53,56,64,67,73,79,89,97,100,105,110,116,121,129,137,143,146,153,159,166,171,176,182,188,194,200,206,212,217,223,229,235,239,245,251,257,263,271,277,283,289,295,301,307,313,318,324,330,336,341,347,353,359,365,371,377,383,389,395,401,407,413,419,425,431,437,443,449,455,461,467,473,479,485,491,497,};
//Ԥ�����ÿ�п�ȣ���ǿ���ã�

uint8 leftExist[HEIGHT], rightExist[HEIGHT];//���߼����ߵĴ���״̬�����Ҫ�ع�����xxxline[]��ֵMISS��
uint8 allFindCnt = 0,allLostCnt = 0, leftLostCnt = 0, rightLostCnt = 0;//������ȫ�ҵ�������������ȫ��ʧ���������߶�ʧ���������߶�ʧ����
uint8 leftDownJump, rightDownJump, leftUpJump, rightUpJump, leftDownStart, rightDownStart, leftUpStart, rightUpStart;
// ����/����/����/���Ϲյ�          ����/����/����/���ϱ߽������ʼ��

RoadTypeEnum RoadType;//��ǰ��·����

int16 ThreeForkCnt = 0; // 0-δ���� 1-����1��,Ŀǰ���������� 2-����2��,Ŀǰ�Ѿ���һ�� 3-����3��,Ŀǰ���������� 4-���������
                          //Ĭ��������ת

float parameterA, parameterB;//���������ѷ�����

imgInfo img_info;//ͼ����������ṹ�壨����ع���������HEIGHT��С�Ŀ��ɽṹ�壩


/**
 * @description: ͼ������ú���
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
 * @description: ��ʼ��ÿһ��ͼ��Ĳ���
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
 * @description: ����ɨ�ߺ���
 *               ��ȡͼ��������ߴ���������������꣬�м�ֵ��ͼ��Χ�Ȳ���
 * @param {*}
 * @return {*}
 */
void basic_searchLine(void)
{
	/**********************************��������*********************************/
	
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
			
			if (tmpTop >= 20)    //��ֹ��һ��ʼ��break
				break;
		}
        width[i] = rightLine[i] - leftLine[i];
        m = midLine[i];
	}

    img_info.top = tmpTop;

	/******************************************************************/
    //               TODO ����������(under construction)
	/*******************************************************************/
	#if 1
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
	int16 start = HEIGHT - 1;
	int16 x, y, tx, ty, s, SearchCompleteFlag, SearchDisruptFlag;

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

    //�����������������
	s = 0;
    x = leftLineSerial[leftLineTmpCnt].x;
    y = start;
    SearchCompleteFlag = 0;
    SearchDisruptFlag = 0;
    while(1)
    {
        //ǿ�ƽ���,��ֹɨ��ͷ�ж�����
//        if()

        //��ʱ����תɨ��
        for(int i = s; ; ++i)
        {
            if(i == 8) i = 0;
            tx = x + dx[i];
			ty = y + dy[i];

            if(ty < 0 || ty >= HEIGHT || tx >= WIDTH)//ɨ���߽�,ֱ���˳�
            {
                SearchCompleteFlag = 1;
                break;
            }
			if(tx < 0)//�ϵ�,x=0
            {
                SearchDisruptFlag = 1;
                y = ty;
                break;
            }
            if(!imageBin[ty][tx])//��ɫ
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


	// while(!leftLine[y])//û���������߶ȵ�ֵ���������Ҫ��������·�����������飩
	// {
	// 	if( x == WIDTH || y == 0) break; //ɨ��ͷ��
	// 	for(int i = s; ; ++i)//��ʱ����ת
	// 	{        
	// 		if(i == 8) i = 0;
	// 		tx = x + dx[i];
	// 		ty = y + dy[i];
	// 		if(tx<0 || ty>=HEIGHT) break; // ���Խ����
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
	// 	s-=2;//����һ��90��
	// }
	
	#endif
	
}

/**
 * @description: ��ȡ�յ����ʼ����� (��ʮ�ּ������յ�Ѱ������)
 * @param {uint8} select_top
 * @param {uint8} select_bottom
 * @return {*}
 */
void basic_getSpecialParams(uint8 select_top, uint8 select_bottom)//TODO ���������������Ĺյ�Ѱ��
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
        if(leftExist[i] && !leftDownStart)     //ɨ����
        {
            leftDownStart = i;
        }
        //���߲���
        else if(rightExist[i] && !rightDownStart)    //ɨ����
        {
            rightDownStart = i;
        }
    }
    
    // leftLastDiff = abs(leftLine[leftDownStart - 1] - leftLine[leftDownStart]);
    // rightLastDiff = abs(rightLine[rightDownStart - 1] - rightLine[rightDownStart]);
    //Ѱ���²�յ�
    for (i = leftDownStart - 1; i > select_top; --i)
    {
        //ֱ���ж�һ����Ĳ�ֵ������ֵ�������Բ��Ǻܺ�
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
    
    //Ѱ���ϲ����
    for (i = select_top; i <= select_bottom; ++i)//�����������ʼλ�ú�ȱʧ���
    {
        //���߲���
        if(leftExist[i] && !leftUpStart)     //ɨ����
        {
            leftUpStart = i;
        }  
        //���߲���
        if(rightExist[i] && !rightUpStart)    //ɨ����
        {
            rightUpStart = i;
        }
    }
    //Ѱ���ϲ�յ�
    for (i = leftUpStart + 2; i < select_bottom; ++i)//��2�ܿ���ֹ�߸���
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
 * @description: �����·��յ������޲��·�ȱʧ�ߵĺ���
 * @param {*}
 * @return {*}
 */
void basic_repairLine(void)//[x] �������ѡ��,��·�²ಹ��,midLine����,[x]��ֳ�����ж�ģ��)
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
    //����
    float left_curvature = get_curvature(leftDownStart, leftDownEnd,LEFT);
    if (left_curvature > -0.1 && left_curvature < 0.4 && leftDownStart - leftDownEnd >= 7)// && leftDownStart <= 32 )
    {
        least_squares(&k, &b, leftDownEnd, leftDownStart, LEFT);
        add_line(k, b, leftDownStart, img_info.bottom, LEFT);
    }

    //����
    float right_curvature = get_curvature(rightDownStart, rightDownEnd, RIGHT);
    if (right_curvature > -0.4 && right_curvature < 0.1 && rightDownStart - rightDownEnd >= 7)// && rightDownStart <= 32 
    {
        least_squares(&k, &b, rightDownEnd, rightDownStart, RIGHT);
        add_line(k, b, rightDownStart, img_info.bottom, RIGHT);
    }

    //�����޲��������
    for(int i = img_info.bottom; i >= min(leftDownStart, rightDownStart); --i)
    {
        midLine[i] = (leftLine[i] + rightLine[i]) / 2;
        // width[i] = rightLine[i] - leftLine[i];
    }
    return ;
}
/**
 * @description: ����ѡ��Χ�����ݶ�Ӧ�÷�Χ����,֧����������
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

    //��״̬������

    //ʮ����״̬
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

    //��״̬������
    

    //���
    if(allFindCnt <= 5 && allLostCnt <= 5)
    {
        if(leftLostCnt > rightLostCnt) //��߶��ı��ұ߶�,����
        {
            #ifdef DEBUG
                std::cout<< "#Turn Left#" <<std::endl;
            #endif
            RoadType = Turn_Left;
            turn_searchLine();
        }
        else //�ұ߱���߶�,����
        {
            #ifdef DEBUG
                std::cout<< "#Turn Right#" <<std::endl;
            #endif
            RoadType = Turn_Right;
            turn_searchLine();
        }
        return ;
    }

    //ֱ��(��������������ж�)
    if((leftDownJump != MISS) + (rightDownJump != MISS) + (leftUpJump != MISS) + (rightUpJump != MISS) <= 1)
    {
        #ifdef DEBUG
                std::cout<< "#Straight#" <<std::endl;
        #endif
        RoadType = Straight;
        basic_repairLine();
        return ;
    }

    // ����ʮ��
    if(leftDownJump != MISS && leftUpJump != MISS && rightDownJump != MISS  && rightUpJump != MISS && RoadType != Three_Fork_Road)
    {
        #ifdef DEBUG
            std::cout<< "#Passing Cross Straightly#" <<std::endl;
        #endif
        RoadType = Cross;
        cross_searchLine();
        return ;
    }
    //б��ʮ��
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
 * @description: �������ɨ��
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
 * @description: ʮ��·��ɨ��
 * @param {*}
 * @return {*}
 */
void cross_searchLine(void)
{
    //����
    float k = (float)(leftLine[leftUpJump] - leftLine[leftDownJump]) / (leftUpJump - leftDownJump);
    float b = (float)leftLine[leftDownJump] - k * leftDownJump;
    // leftLine[leftDownJump] = k * leftDownJump + b;

    add_line(k, b, leftUpJump, leftDownJump, LEFT);

    //����
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
 * @description: ��ȡ��ǰλ�ú�����λ�õ�ƫ����ݸ�PID����
 * @param {*}
 * @return {*}
 */
void get_error(void)//TODO ʵ�ֶ�̬����ǰհ
{
    int avePos, predictedPass;
    float spd = 2.5;// = get_speed() ֮��д����speed��̬ѡ��ǰհ��ģʽ
    predictedPass = min(spd * 2, img_info.top - 4); //֮�����ʵ�����д����ϵʽ
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
 * @description: �ж���/��/���ߵ�������
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
            for (int j = select_top; j < select_bottom; ++j)//�ӵ�10�п�ʼ����ֹ��������ϵ���Ӱ���ж�
            {
                delta = leftLine[j + 1] - leftLine[j];       //left�ߵ�ƫ��
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
                delta = midLine[j + 1] - midLine[j];       //left�ߵ�ƫ��
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
                delta = rightLine[j + 1] - rightLine[j];       //left�ߵ�ƫ��
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
 * @description: �����������
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
    //����ķ��ű�ʾ����
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
 * @description: ���ߣ���Χ���ұգ�
 * @param {float} k ����б�� ͨ��least_squares()������ȡ
 * @param {float} b ����ƫ�� ͨ��least_squares()������ȡ
 * @param {uint8} select_top ��Χ��㣨���������ڣ�
 * @param {uint8} select_bottom ��Χ�յ㣨�������ڣ�
 * @param {LineTypeEnum} type ���ߵĶ������� {LEFT/MID/RIGHT}
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
 * @description: ��С���˷����ֱ��
 *               ע�⣺�����һ��ͼ�������ϵ��һ���������Ͻ�Ϊԭ�㣬����Ϊx_t������Ϊy_t��
 *               �� y_t = $k$ * x_t + $b$ ������ԭ����ϵֱ�Ӵ��뵱ǰͼ��߶� h_0 ���������Ӧ�� ������ w_0
 * @param {float *} k ����ֱ��б��
 * @param {float *} b ����ֱ��ƫ��
 * @param {uint8} select_top ѡ����Χ�Ŀ�ʼ���߶ȷ�Χ��
 * @param {uint8} select_bottom ѡ����Χ�Ľ������߶ȷ�Χ��
 * @param {LeastSquaresObjectEnum} type ��Ҫ��ϵ�ֱ�����ͣ���߽�/�ұ߽�/���ߣ�
 * @return {*} �޷���ֵ��ͨ������ $k$ �� $b$ �õ�ֱ�߲���
 */
void least_squares(float * k, float * b, uint8 select_top, uint8 select_bottom, LineTypeEnum type)
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
                sumxy += i * leftLine[i];
                sumy  += leftLine[i];
                sumy2 += leftLine[i] * leftLine[i];
            }
            break;
        case MID:
            for (int i = select_top; i <= select_bottom; ++i)
            {
                sumx  += i; //i ��Ϊx��ֵ
                sumx2 += i * i;
                sumxy += i * midLine[i];
                sumy  += midLine[i];
                sumy2 += midLine[i] * midLine[i];
            }
            break;
        case RIGHT:
            for (int i = select_top; i <= select_bottom; ++i)
            {
                sumx  += i; //i ��Ϊx��ֵ
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
            sumY += midLine[i];
        }
        for (i = startline2; i < endline2; i++)
        {
            sumX += i;
            sumY += midLine[i];
        }
        averageX = (float)sumX / (sumlines1 + sumlines2);     //x��ƽ��ֵ
        averageY = (float)sumY / (sumlines1 + sumlines2);     //y��ƽ��ֵ
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
    else if (type == 1)     //�������
    {
        /**����sumX sumY**/
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
        averageX = (float)sumX / (sumlines1 + sumlines2);     //x��ƽ��ֵ
        averageY = (float)sumY / (sumlines1 + sumlines2);     //y��ƽ��ֵ
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
    else if (type == 2)         //�������
    {
        /**����sumX sumY**/
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
        averageX = (float)sumX / (sumlines1 + sumlines2);     //x��ƽ��ֵ
        averageY = (float)sumY / (sumlines1 + sumlines2);     //y��ƽ��ֵ
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
