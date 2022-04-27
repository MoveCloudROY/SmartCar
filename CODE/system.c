/*
 * system.c
 *
 *  Created on: 2022年4月9日
 *      Author: ROY1994
 */
#include "headfile.h"
#include "data.h"
#include "motor.h"
#include "steer.h"
#include "pid.h"
#include "ImageDeal.h"
#include "ImagePreDeal.h"
#include "key.h"
#include "system.h"
#include "ui.h"



#define __DEBUG_IPS_ON__

extern ConstDataTypeDef ConstData;
extern uint8_t imageBin[HEIGHT][WIDTH];

extern PID PID_L, PID_R;

extern uint16 cpu1_5ms_flag;

extern uint16 delay_20ms_flag, delay_100ms_flag, delay_1000ms_flag;

void car_init(void)
{
    gpio_init(P20_8, GPO, 0, PUSHPULL);//状态提示灯
    gpio_init(P21_4, GPO, 0, PUSHPULL);//DEBUG灯
    gpio_init(P20_9, GPO, 1, PUSHPULL);//DEBUG灯
    gpio_init(P21_5, GPO, 1, PUSHPULL);//DEBUG灯

#ifdef __DEBUG_IPS_ON__
    ips200_init();
#endif
    key_init();

    uart_init(UART_0, 2000000, UART0_TX_P14_0, UART0_RX_P14_1);//图像发送串口

    systick_delay_ms(STM0, 500);//延时0.5ms
    seekfree_wireless_init();

    data_set();
    mt9v03x_init();
    servo_init();
    motor_init();

    servo_set(ConstData.kServoMid);

//  int t = 1100;
//  pwm_duty(MOTOR_RA, 5000+t);
//  pwm_duty(MOTOR_RB, 5000-t);

    while(startKey_read());

    pit_init(CCU6_0, PIT_CH0, 5000);
    pit_init(CCU6_0, PIT_CH1, 20000);
}

void car_backstage(void)
{

}
void img_backstage(void)
{
    if(mt9v03x_finish_flag)
    {
        if(cpu1_5ms_flag)
        {
//            img_preProcess(CUT);
            img_preProcess(OTSU);

            img_preProcess(MORPH_EROSION);

            img_process();
            gpio_toggle(P20_9);
            cpu1_5ms_flag = 0;
        }
        if(delay_20ms_flag)
        {
//            gpio_set(P21_4,1);
            gpio_toggle(P21_4);

#ifdef __DEBUG_IPS_ON__
            draw_image();
            draw_line();
#endif

//            a_sendimg_wifi(UART_0, mt9v03x_image, MT9V03X_W, MT9V03X_H);
//            my_sendimg_wifi(UART_0, imageBin, MT9V03X_W, MT9V03X_H);

            delay_20ms_flag = 0;
        }
        mt9v03x_finish_flag = 0;
    }
}
