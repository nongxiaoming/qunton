/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/* Hardware  includes. */
#include "board.h"
#include "stm32f4xx.h"
#include "drv_led.h"
#include "drv_mpu6050.h"


int main(void)
{
  board_init();
	/* Create the LED Task. */

	//MPU6050_Test_Init();
	/* Start the scheduler. */
	vTaskStartScheduler();
 
	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for( ;; );
}

