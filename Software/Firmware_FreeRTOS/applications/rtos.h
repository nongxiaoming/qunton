#ifndef __RTOS_H

#include "types.h"

#ifdef OS_USING_FREERTOS
#include "FreeRTOS.h"
#include "semphr.h"
/* task */
#define os_task_t xTaskHandle
/* queue */
#define os_queue_t  xQueueHandle
#define OS_QueueCreate(queue_len,item_size) xQueueGenericCreate( queue_len, item_size, queueQUEUE_TYPE_BASE )

/* os */
#define OS_Delay(x) vTaskDelay(x)


#endif


#endif /* endif __RTOS_H */
