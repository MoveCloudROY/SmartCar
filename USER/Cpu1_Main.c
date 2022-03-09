/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/

#include "headfile.h"
#pragma section all "cpu1_dsram"
//���������#pragma section all restore���֮���ȫ�ֱ���������CPU1��RAM��

extern uint16 cpu1_5ms_flag;
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern uint8 imageBin[HEIGHT][WIDTH];
extern uint16 delay_20ms_flag, delay_100ms_flag, delay_1000ms_flag;
void core1_main(void)
{
	disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    //�û��ڴ˴����ø��ֳ�ʼ��������





	//�ȴ����к��ĳ�ʼ�����
	IfxCpu_emitEvent(&g_cpuSyncEvent);
	IfxCpu_waitEvent(&g_cpuSyncEvent, 0xFFFF);
    enableInterrupts();
    while (TRUE)
    {
        if(mt9v03x_finish_flag)
        {
            if(cpu1_5ms_flag)
            {
                img_preProcess(OTSU);
//                img_process();

                cpu1_5ms_flag = 0;
            }
            if(delay_100ms_flag)
            {
//                gpio_set(P21_4,1);
                gpio_toggle(P21_4);
                my_sendimg_03x(UART_0, mt9v03x_image, MT9V03X_W, MT9V03X_H);

                delay_100ms_flag = 0;
            }
            mt9v03x_finish_flag = 0;
        }
    }
}



#pragma section all restore
