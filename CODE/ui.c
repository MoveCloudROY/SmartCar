/*
 * ui.c
 *
 *  Created on: 2022��4��17��
 *      Author: ROY1994
 */
#include "ui.h"

/*  ��ɫ������seekfreee_font.h,����ֱ����
#define RED             0xF800  //��ɫ
#define BLUE            0x001F  //��ɫ
#define YELLOW          0xFFE0  //��ɫ
#define GREEN           0x07E0  //��ɫ
#define WHITE           0xFFFF  //��ɫ
#define BLACK           0x0000  //��ɫ
#define GRAY            0X8430  //��ɫ
#define BROWN           0XBC40  //��ɫ
#define PURPLE          0XF81F  //��ɫ
#define PINK            0XFE19  //��ɫ
*/

extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern uint8 imageBin[HEIGHT][WIDTH];
extern RowInfoTypedef rowInfo[HEIGHT];
extern ImgInfoTypedef imgInfo;

void draw_image(void)
{
    ips200_displayimage032(imageBin, WIDTH, HEIGHT);
}
void draw_line(void)
{
    //���߼����ұ���
    for (int i = imgInfo.top; i < imgInfo.bottom; i++)
    {
        if(rowInfo[i].rightLine < WIDTH && rowInfo[i].rightLine >= 0)
//            imgColor.at<Vec3b>(i, ) = RED;
            ips200_drawpoint(rowInfo[i].rightLine, i, RED);
        if(rowInfo[i].leftLine < WIDTH && rowInfo[i].leftLine >= 0)
            ips200_drawpoint(rowInfo[i].leftLine, i, BLUE);
        if(rowInfo[i].midLine < WIDTH && rowInfo[i].midLine >= 0)
            ips200_drawpoint(rowInfo[i].midLine, i, GREEN);
    }
    //��Ч������
    for (int j = 0; j < WIDTH; j++)
    {
        ips200_drawpoint(j, imgInfo.bottom, YELLOW);
        ips200_drawpoint(j, imgInfo.top, YELLOW);
    }
}
