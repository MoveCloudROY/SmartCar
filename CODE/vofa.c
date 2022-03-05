#include "vofa.h"

float data[8];

void vofa_sendFloat(float f)
{
    uint8*  p;
    p = (uint8*)&f;
    uart_putchar(WIRELESS_UART, *(p+0));
    uart_putchar(WIRELESS_UART, *(p+1));
    uart_putchar(WIRELESS_UART, *(p+2));
    uart_putchar(WIRELESS_UART, *(p+3));
}
//void vofa_sendInt(int x)
//{
//    uint8*  p;
//    p = (uint8*)&x;
//    uart_putchar(WIRELESS_UART, *(p+0));
//    uart_putchar(WIRELESS_UART, *(p+1));
//    uart_putchar(WIRELESS_UART, *(p+2));
//    uart_putchar(WIRELESS_UART, *(p+3));
//}
void vofa_sendTail(void)
{
    uart_putchar(WIRELESS_UART, 0x00);uart_putchar(WIRELESS_UART, 0x00);
    uart_putchar(WIRELESS_UART, 0x80);uart_putchar(WIRELESS_UART, 0x7f);
}
