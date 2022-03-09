/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,逐飞科技
 * All rights reserved.
 * 技术讨论QQ群：三群：824575535
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file       		main
 * @company	   		成都逐飞科技有限公司
 * @author     		逐飞科技(QQ3184284598)
 * @version    		查看doc内version文件 版本说明
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/

#include "headfile.h"
#pragma section all "cpu1_dsram"
//将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

extern uint16 cpu1_5ms_flag;
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern uint8 imageBin[HEIGHT][WIDTH];
extern uint16 delay_20ms_flag, delay_100ms_flag, delay_1000ms_flag;
void core1_main(void)
{
	disableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    //用户在此处调用各种初始化函数等





	//等待所有核心初始化完毕
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
