/*
 * key.h
 *
 *  Created on: 2022Äê4ÔÂ10ÈÕ
 *      Author: ROY1994
 */

#ifndef CODE_KEY_H_
#define CODE_KEY_H_
#include "headfile.h"

#define START_KEY   P22_3
#define KEY_2       P22_2
#define KEY_3       P22_1
#define KEY_4       P22_0

void key_init(void);
uint8 startKey_read(void);

#endif /* CODE_KEY_H_ */
