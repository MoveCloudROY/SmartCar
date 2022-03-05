/*
 * @Author: ROY1994
 * @Date: 2022-02-25 20:06:39
 * @LastEditors: ROY1994
 * @LastEditTime: 2022-03-05 11:41:06
 * @FilePath: \myImageDeal_v0.1\stack.cpp
 * @Description: 栈,用于元素处理
 */
#include "stack.h"


void stack_init(stack * obj)
{
    obj->top = 0;
    return ;
}
void stack_push(stack * obj, RoadTypeEnum x)
{
    obj->data[++(obj->top)];
    return ;
}
void stack_pop(stack * obj)
{
    --(obj->top);
    return ;
}
RoadTypeEnum stack_getTop(stack * obj)
{
    return obj->data[obj->top];
}
