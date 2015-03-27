#include "stm32f4xx.h"

void LED_GPIO_Init(void)
{
 /* LED0 <---> PE0
	* LED1 <---> PE1
	* LED2 <---> PE2
	* LED3 <---> PE3
	**/

    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    /* output setting */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
	  GPIO_SetBits(GPIOE, 0x0f);
}

void led_on(unsigned int id)
{
	if(id < 4)
 GPIO_ResetBits(GPIOE, 0x01<<id);
}

void led_off(unsigned int id)
{
if(id < 4)
GPIO_SetBits(GPIOE, 0x01<<id);
}
