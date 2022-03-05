/*
 * @Author: ROY1994
 * @Date: 2022-02-25 20:06:45
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-03-05 11:41:00
 * @FilePath: \myImageDeal_v0.1\stack.h
 * @Description: Õ»-Í·ÎÄ¼þ
 */

#ifndef _STACK_H_
#define _STACK_H_

typedef enum _RoadTypeEnum{Straight, Cross, Slope, S_bend, Big_bend, Reflection, Round_About, Starting_Line, Three_Fork_Road, Turn_Left, Turn_Right} RoadTypeEnum;

typedef struct _stack
{
    RoadTypeEnum data[15];
    int top;
}stack;

void stack_init(stack * obj);
void stack_push(stack * obj,RoadTypeEnum x);
void stack_pop(stack * obj);
RoadTypeEnum stack_getTop(stack * obj);


#endif /*_STACK_H_*/

