#ifndef __USART_H__
#define __USART_H__
#include "stm32f4xx.h"
#include <stdio.h>

#define UART_ENABLE_IRQ(n)            NVIC_EnableIRQ((n))
#define UART_DISABLE_IRQ(n)           NVIC_DisableIRQ((n))

int USART_HW_Init(void);
int putc(int ch,FILE * f);
 int stm32_putc(char c);
 int stm32_getc(void);
 void Debug(char* str);
 void kprintf(const char *fmt, ...);
#endif
