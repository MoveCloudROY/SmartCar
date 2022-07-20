/*
 * buzzer.c
 *
 *  Created on: 2022Äê7ÔÂ13ÈÕ
 *      Author: ROY1994
 */
#include "buzzer.h"

extern SystemDataTypedef SystemData;

void buzzer_init(void)
{
    gpio_init(BUZZER_PIN, GPO, 0, PUSHPULL);
}
void call_buzzer(void)
{
    SystemData.isBuzzerOn = 'T';
}
void deal_buzzer(void)
{
    if(SystemData.isBuzzerOn == 'T')
    {
        gpio_set(BUZZER_PIN, 1);
        systick_delay_ms(STM0, 600);
        gpio_set(BUZZER_PIN, 0);
        SystemData.isBuzzerOn = 'F';
    }
}
