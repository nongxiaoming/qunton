
#include "drv_usart.h"
#include "freeRTOS.h"

/* UART GPIO define. */
#define UART1_GPIO_TX       GPIO_Pin_9
#define UART1_TX_PIN_SOURCE GPIO_PinSource9
#define UART1_GPIO_RX       GPIO_Pin_10
#define UART1_RX_PIN_SOURCE GPIO_PinSource10
#define UART1_GPIO          GPIOA
#define UART1_GPIO_RCC      RCC_AHB1Periph_GPIOA
#define RCC_APBPeriph_UART1 RCC_APB2Periph_USART1
#define UART1_TX_DMA        DMA1_Channel4
#define UART1_RX_DMA        DMA1_Channel5




 int stm32_putc(char c)
{

    while (!(USART1->SR & USART_FLAG_TXE));
    USART1->DR = c;

    return 1;
}

 int stm32_getc(void)
{
    int ch;


    ch = -1;
    if (USART1->SR & USART_FLAG_RXNE)
    {
        ch = USART1->DR & 0xff;
    }

    return ch;
}

void USART1_IRQHandler(void)
{
    /* enter interrupt */
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        //rt_hw_serial_isr(&serial1, RT_SERIAL_EVENT_RX_IND);
        /* clear interrupt */
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        /* clear interrupt */
        USART_ClearITPendingBit(USART1, USART_IT_TC);
    }

}


static void RCC_Configuration(void)
{
    /* Enable UART1 GPIO clocks */
    RCC_AHB1PeriphClockCmd(UART1_GPIO_RCC, ENABLE);
    /* Enable UART1 clock */
    RCC_APB2PeriphClockCmd(RCC_APBPeriph_UART1, ENABLE);
}

static void GPIO_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

    /* Configure USART1 Rx/tx PIN */
    GPIO_InitStructure.GPIO_Pin = UART1_GPIO_RX | UART1_GPIO_TX;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

    /* Connect alternate function */
    GPIO_PinAFConfig(UART1_GPIO, UART1_TX_PIN_SOURCE, GPIO_AF_USART1);
    GPIO_PinAFConfig(UART1_GPIO, UART1_RX_PIN_SOURCE, GPIO_AF_USART1);


}
static void USART_Configuration()
{
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
     /* enable rx irq */
    UART_ENABLE_IRQ(USART1_IRQn);
     /* enable interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    /* Enable USART */
    USART_Cmd(USART1, ENABLE);
	
}
static void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Enable the USART1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

int USART_HW_Init(void)
{

    RCC_Configuration();
    GPIO_Configuration();
    NVIC_Configuration();
    USART_Configuration();

    return 0;
}

int putc(int ch, FILE * f)
{
   stm32_putc((char)ch);
	return  0;
}
void Debug(char* str)
{
 while(*str)
 {
  stm32_putc(*str);
	str++;
 }
}

#include <stdarg.h>
/**
 * This function will print a formatted string on system console
 *
 * @param fmt the format
 */
void kprintf(const char *fmt, ...)
{
     va_list args;
     uint32_t length;
    static char log_buf[256];

    va_start(args, fmt);
    /* the return value of vsnprintf is the number of bytes that would be
     * written to buffer had if the size of the buffer been sufficiently
     * large excluding the terminating null byte. If the output string
     * would be larger than the rt_log_buf, we have to adjust the output
     * length. */
    length = vsnprintf(log_buf, sizeof(log_buf) - 1, fmt,args);
    if (length > 256 - 1)
        length = 256 - 1;
     Debug(log_buf);
   // rt_hw_console_output(rt_log_buf);

    va_end(args);
}
