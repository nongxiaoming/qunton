
#include "stm32f4xx.h"
#include "board.h"
#include "drv_usart.h"
/**
 * @addtogroup STM32
 */

/*@{*/

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void NVIC_Configuration(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

/**
 * This function will initial STM32 board.
 */
void board_init()
{
    /* NVIC Configuration */
    NVIC_Configuration();
	  USART_HW_Init();
}

/*@}*/
