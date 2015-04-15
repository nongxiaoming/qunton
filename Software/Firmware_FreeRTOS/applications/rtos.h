#ifndef __RTOS_H

#include "types.h"
#define OS_USING_FREERTOS
#ifdef OS_USING_FREERTOS
#include "FreeRTOS.h"
#include "semphr.h"

/* task */
#define os_task_t xTaskHandle

/* semaphore */
#define os_sema_t SemaphoreHandle_t
/* queue */
#define os_queue_t  xQueueHandle
#define OS_QueueCreate(queue_len,item_size) xQueueGenericCreate( queue_len, item_size, queueQUEUE_TYPE_BASE )

/* os */
#define OS_Delay(x) vTaskDelay(x)
#define OS_Assert(x)  configASSERT(x)

/* mem */
#define OS_Free(x)   vPortFree(x)
#define OS_Malloc(size) pvPortMalloc(size)
#endif


#endif /* endif __RTOS_H */
