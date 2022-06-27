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
#include "shell.h"
#include "vt100.h"


//#define __DEBUG_IPS_ON__

extern ConstDataTypeDef ConstData;
extern uint8_t imageBin[HEIGHT][WIDTH];

extern PID PID_L, PID_R, PID_Servo;
extern float speedL, speedR;
extern int steer_pwm;
extern ImgInfoTypedef imgInfo;

extern uint16 cpu0_5ms_flag, cpu0_1000ms_flag, cpu1_5ms_flag;

extern uint16 delay_20ms_flag, delay_100ms_flag, delay_1000ms_flag;

extern PassDisTypedef passDis;

volatile SystemStatusTypedef Global = {
        .cpu0_usage = 0,
        .cpu1_usage = 0,
};

void car_init(void)
{
    gpio_init(P20_8, GPO, 0, PUSHPULL);//状态提示灯
    gpio_init(P21_4, GPO, 0, PUSHPULL);//DEBUG灯
    gpio_init(P20_9, GPO, 1, PUSHPULL);//DEBUG灯
    gpio_init(P21_5, GPO, 1, PUSHPULL);//DEBUG灯

#ifdef __DEBUG_IPS_ON__
    ips200_init();
#endif

    shell_init();

    key_init();

    uart_init(UART_0, 2000000, UART0_TX_P14_0, UART0_RX_P14_1);//图像发送串口

    systick_delay_ms(STM0, 500);//延时0.5ms


    data_set();
    mt9v03x_init();
    servo_init();
    motor_init();

    servo_set(ConstData.kServoMid);

//  int t = 1100;
//  pwm_duty(MOTOR_RA, 5000+t);
//  pwm_duty(MOTOR_RB, 5000-t);
    seekfree_wireless_init();

    while(startKey_read());

    pit_init(CCU6_0, PIT_CH0, 5000);
    pit_init(CCU6_0, PIT_CH1, 20000);

    vt_clearall();
}

void car_backstage(void)
{
//    shell_run();
    vt_hide_cursor();

    if(cpu0_5ms_flag)
    {
//        vt_set_font_color(VT_F_RED);
//        vt_draw_str("aBc");
//        vt_move_left(3);
        car_statusbar();
        cpu0_5ms_flag = 0;
    }
    if(cpu0_1000ms_flag)
    {
        vt_clearall();
        cpu0_1000ms_flag = 0;
    }
}
void img_backstage(void)
{
    if(mt9v03x_finish_flag)
    {
        if(cpu1_5ms_flag)
        {
//            img_preProcess(CUT);
            img_preProcess(OTSU);

//            img_preProcess(MORPH_EROSION);

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
//            my_sendimg_wifi(UART_0, mt9v03x_image, MT9V03X_W, MT9V03X_H);

            delay_20ms_flag = 0;
        }
        mt9v03x_finish_flag = 0;
    }
}


void car_statusbar(void)
{
    char ss[100] = "";
    // 画盒子
//    vt_draw_box(1, 1, 20, 80, ' ', ' ', ' ');
    //标题
    vt_set_font_color(VT_F_CYAN);
    vt_draw_str_at(2, 34, "Car Status");

    // 道路类型
    vt_set_font_color(VT_F_RED);
    vt_draw_str_at(3, 2, "RoadType: ");
    vt_set_font_color(VT_F_WHITE);
    switch (imgInfo.RoadType)
    {
        case Road_None:         vt_draw_str_at(3, 14, "Road_None    ");     break;
        case Cross:             vt_draw_str_at(3, 14, "Cross        ");     break;
        case Circle_L:          vt_draw_str_at(3, 14, "Circle_L     ");     break;
        case Circle_R:          vt_draw_str_at(3, 14, "Circle_R     ");     break;
        case Straight:          vt_draw_str_at(3, 14, "Straight     ");     break;
        case Slope:             vt_draw_str_at(3, 14, "Slope        ");     break;
        case S_bend:            vt_draw_str_at(3, 14, "S_bend       ");     break;
        case Big_bend:          vt_draw_str_at(3, 14, "Big_bend     ");     break;
        case Reflection:        vt_draw_str_at(3, 14, "Reflection   ");     break;
        case Starting_Line:     vt_draw_str_at(3, 14, "Starting_Line");     break;
        case Fork_In:           vt_draw_str_at(3, 14, "Fork_In      ");     break;
        case Fork_Out:          vt_draw_str_at(3, 14, "Fork_Out     ");     break;
        case Turn_Left:         vt_draw_str_at(3, 14, "Turn_Left    ");     break;
        case Turn_Right:        vt_draw_str_at(3, 14, "Turn_Right   ");     break;
        default:                                                            break;
    }

    // 输出环岛状态
    vt_set_font_color(VT_F_RED);
    vt_draw_str_at(3, 30, "CircleStatus: ");
    vt_set_font_color(VT_F_WHITE);
    switch (imgInfo.CircleStatus)
    {
        case CIRCLE_OUT:            vt_draw_str_at(3, 45, "CIRCLE_OUT    ");    break;
        case CIRCLE_IN:             vt_draw_str_at(3, 45, "CIRCLE_IN     ");    break;
        case CIRCLE_FIND:           vt_draw_str_at(3, 45, "CIRCLE_FIND   ");    break;
        case CIRCLE_PASSING:        vt_draw_str_at(3, 45, "CIRCLE_PASSING");    break;
        case CIRCLE_OFF:            vt_draw_str_at(3, 45, "CIRCLE_OFF    ");    break;
        default:                                                                break;
    }

    // 输出左轮速度
    vt_set_font_color(VT_F_RED);
    vt_draw_str_at(4, 2, "LeftSpeed: ");
    vt_set_font_color(VT_F_WHITE);
    sprintf(ss, "%.2f", speedL);
    vt_draw_str_at(4, 14, ss);

    // 输出右轮速度
    vt_set_font_color(VT_F_RED);
    vt_draw_str_at(4, 24, "RightSpeed: ");
    vt_set_font_color(VT_F_WHITE);
    sprintf(ss, "%.2f", speedR);
    vt_draw_str_at(4, 36, ss);

    // 输出平均速度
    vt_set_font_color(VT_F_RED);
    vt_draw_str_at(4, 48, "AveSpeed: ");
    vt_set_font_color(VT_F_WHITE);
    sprintf(ss, "%.2f", (speedL + speedR) / 2.0);
    vt_draw_str_at(4, 60, ss);
    // 输出舵机打角
}
