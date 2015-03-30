#include "sensors.h"


//! The list of queue handles
static  os_queue_t queues[SENSOR_LAST];
static int32_t max_gyro_rate;

//! Initialize the sensors interface
int32_t SENSORS_Init()
{
	for (uint32_t i = 0; i < SENSOR_LAST; i++)
		queues[i] = NULL;

	return 0;
}

//! Register a sensor with the PIOS_SENSORS interface
int32_t SENSORS_Register(enum sensor_type type, os_queue_t queue)
{
	if(queues[type] != NULL)
		return -1;

	queues[type] = queue;

	return 0;
}

//! Get the data queue for a sensor type
 os_queue_t SENSORS_GetQueue(enum sensor_type type)
{
	if (type >= SENSOR_LAST)
		return NULL;

	return queues[type];
}

//! Set the maximum gyro rate in deg/s
void SENSORS_SetMaxGyro(int32_t rate)
{
	max_gyro_rate = rate;
}

//! Get the maximum gyro rate in deg/s
int32_t SENSORS_GetMaxGyro()
{
		return max_gyro_rate;
}
