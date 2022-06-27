/*
 * @Author: ROY1994
 * @Date: 2022-02-04 13:40:02
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-03-05 11:40:28
 * @FilePath: \myImageDeal_v0.1\ImagePreDeal.h
 * @Description: 预处理头文件定义
 */
#ifndef IMAGE_PRE_DEAL_H
#define IMAGE_PRE_DEAL_H


//#define ON_PC//移植记得注释

#ifdef ON_PC

    #include <opencv2/opencv.hpp>
    #include <iostream>
    #include <highgui.hpp>
    #include <core.hpp>
    #include <cmath>

#else

    #include "headfile.h"

#endif

#define WIDTH                           187                                     //图像宽
#define HEIGHT                          120                                     //图像高


#define min(x, y)                       (((x) < (y)) ? (x) : (y))
#define max(x, y)                       (((x) > (y)) ? (x) : (y))
//#define abs(x)                          (((x) > 0) ? (x) : (-(x)))

#define TH_SOBEL                        128                                     //Sobel算子阈值
#define k_sauvola                       0.04                                    //sauvola算法 参数k
#define r_sauvola                       128.0                                   //sauvola算法 参数r
#define k2_sauvola                      0.0036                                  //sauvola算法 参数k方
#define r2_sauvola                      16384.0                                 //sauvola算法 参数r方
//#define kernel_size_sauvola             10 
#define kernel_sizeby2_sauvola          5                                       //sauvola算法 局部滑窗半径（直径*2+1）

#define kernel_sizeby2_medianfilter     5                                       //中值滤波算法 滑窗半径
#define kernel_size_medianfilter        11                                      //中值滤波算法 滑窗直径
#define kernel_2_size_medianfilter      121                                     //中值滤波算法 滑窗面积
#define kernel_mid_medianfilter         61                                      //中值滤波算法 中位数排名
#define N 256                                                                   //0~255移动至1~256后的上限
#define lowbit(x) ((x)&(-(x)))                                                  //树状数组的lowbit函数

#define kernel_sizeby2_morph               2                                    //形态学处理滑窗半径
#define kernel_size_morph                  5                                    //形态学处理滑窗直径

typedef enum _PreDealMethodEnum{OTSU,OTSU2D,SAUVOLA,SOBEL,GAUSSIAN_FILTER,MEDIAN_FILTER,MORPH_EROSION,MORPH_DILITION,MY_MORPH_OPEN,MY_MORPH_CLOSE} PreDealMethodEnum;



void img_preProcess(PreDealMethodEnum method);
void compress(void);
uint8_t otsu(void);
void sauvola(void);
void median_filter(void);
void sobel(void);
void morph_erosion(void);
void morph_dilition(void);
void morph_open(void);
void morph_close(void);

#endif /*IMAGE_PRE_DEAL_H*/
