/*
 * system.h
 *
 *  Created on: 2022Äê4ÔÂ9ÈÕ
 *      Author: ROY1994
 */

#ifndef CODE_SYSTEM_H_
#define CODE_SYSTEM_H_


#define RUN_TIME(func)                                      \
do{                                                         \
    char sss[50];                                           \
    systick_start(STM0);                                    \
    func;                                                   \
    float t = systick_getval(STM0);                         \
    vt_set_font_color(VT_F_RED);                            \
    vt_draw_str_at(15, 2,"func ");                          \
    vt_set_bg_color(VT_F_PURPLE);                           \
    vt_draw_str_at(16, 2, #func);                           \
    vt_set_font_color(VT_F_RED);                            \
    sprintf(sss, " takes %.3fms in cpu", t / 100000.0f);    \
    vt_draw_str_at(16, 14, sss);                            \
    vt_set_bg_color(VT_F_CYAN);                             \
    sprintf(sss, "%d.\n\r", IfxCpu_getCoreIndex());         \
    vt_draw_str_at(16, 44, sss);                            \
}while(0)

typedef struct _SystemStatusTypedef
{
    int cpu0_usage, cpu1_usage;

    int seconds;
}SystemStatusTypedef;


void car_init(void);
void car_backstage(void);
void img_backstage(void);
void car_statusbar(void);
#endif /* CODE_SYSTEM_H_ */
