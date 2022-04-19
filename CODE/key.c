/*
 * key.c
 *
 *  Created on: 2022��4��10��
 *      Author: ROY1994
 */
#include "headfile.h"
#include "key.h"

void key_init()
{
    gpio_init(START_KEY, GPI, 0, PUSHPULL);
}

uint8 startKey_read()
{
    return gpio_get(START_KEY);
}

