/*
 * @Author: ROY1994
 * @Date: 2022-02-04 13:40:02
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-03-05 11:40:28
 * @FilePath: \myImageDeal_v0.1\ImagePreDeal.h
 * @Description: Ԥ����ͷ�ļ�����
 */
#ifndef IMAGE_PRE_DEAL_H
#define IMAGE_PRE_DEAL_H


//#define ON_PC//��ֲ�ǵ�ע��

#ifdef ON_PC

    #include <opencv2/opencv.hpp>
    #include <iostream>
    #include <highgui.hpp>
    #include <core.hpp>
    #include <cmath>

#else

    #include "headfile.h"

#endif

#define WIDTH                           187                                     //ͼ���
#define HEIGHT                          120                                     //ͼ���


#define min(x, y)                       (((x) < (y)) ? (x) : (y))
#define max(x, y)                       (((x) > (y)) ? (x) : (y))
//#define abs(x)                          (((x) > 0) ? (x) : (-(x)))

#define TH_SOBEL                        128                                     //Sobel������ֵ
#define k_sauvola                       0.04                                    //sauvola�㷨 ����k
#define r_sauvola                       128.0                                   //sauvola�㷨 ����r
#define k2_sauvola                      0.0036                                  //sauvola�㷨 ����k��
#define r2_sauvola                      16384.0                                 //sauvola�㷨 ����r��
//#define kernel_size_sauvola             10 
#define kernel_sizeby2_sauvola          5                                       //sauvola�㷨 �ֲ������뾶��ֱ��*2+1��

#define kernel_sizeby2_medianfilter     5                                       //��ֵ�˲��㷨 �����뾶
#define kernel_size_medianfilter        11                                      //��ֵ�˲��㷨 ����ֱ��
#define kernel_2_size_medianfilter      121                                     //��ֵ�˲��㷨 �������
#define kernel_mid_medianfilter         61                                      //��ֵ�˲��㷨 ��λ������
#define N 256                                                                   //0~255�ƶ���1~256�������
#define lowbit(x) ((x)&(-(x)))                                                  //��״�����lowbit����

#define kernel_sizeby2_morph               2                                    //��̬ѧ�������뾶
#define kernel_size_morph                  5                                    //��̬ѧ������ֱ��

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
