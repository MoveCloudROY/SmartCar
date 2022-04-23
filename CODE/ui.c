/*
 * ui.c
 *
 *  Created on: 2022年4月17日
 *      Author: ROY1994
 */
#include "ui.h"

/*  颜色定义在seekfreee_font.h,这里直接用
#define RED             0xF800  //红色
#define BLUE            0x001F  //蓝色
#define YELLOW          0xFFE0  //黄色
#define GREEN           0x07E0  //绿色
#define WHITE           0xFFFF  //白色
#define BLACK           0x0000  //黑色
#define GRAY            0X8430  //灰色
#define BROWN           0XBC40  //棕色
#define PURPLE          0XF81F  //紫色
#define PINK            0XFE19  //粉色
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
    //中线及左右边线
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
    //有效区域标记
    for (int j = 0; j < WIDTH; j++)
    {
        ips200_drawpoint(j, imgInfo.bottom, YELLOW);
        ips200_drawpoint(j, imgInfo.top, YELLOW);
    }
}
