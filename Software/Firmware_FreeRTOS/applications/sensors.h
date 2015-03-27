#ifndef __SENSOR_H
#define __SENSOR_H
#include "stdint.h"
#include "FreeRTOS.h"
#include "queue.h"

//! Pios sensor structure for generic gyro data
struct sensor_gyro_data {
	float x;
	float y; 
	float z;
	float temperature;
};

//! Pios sensor structure for generic accel data
struct sensor_accel_data {
	float x;
	float y; 
	float z;
	float temperature;
};

//! Pios sensor structure for generic mag data
struct sensor_mag_data {
	float x;
	float y; 
	float z;
};

//! Pios sensor structure for generic baro data
struct sensor_baro_data {
	float temperature;
	float pressure;
	float altitude;
};

//! The types of sensors this module supports
enum sensor_type
{
	SENSOR_ACCEL,
	SENSOR_GYRO,
	SENSOR_MAG,
	SENSOR_BARO,
	SENSOR_LAST
};

//! Structure to register the data
struct sensor_registration {
	enum sensor_type type;
	xQueueHandle queue;
};

//! Initialize the PIOS_SENSORS interface
int32_t SENSORS_Init(void);

//! Register a sensor with the PIOS_SENSORS interface
int32_t SENSORS_Register(enum sensor_type type, xQueueHandle queue);

//! Get the data queue for a sensor type
struct xQueueHandle PIOS_SENSORS_GetQueue(enum sensor_type type);

//! Set the maximum gyro rate in deg/s
void SENSORS_SetMaxGyro(int32_t rate);

//! Get the maximum gyro rate in deg/s
int32_t SENSORS_GetMaxGyro(void);

#endif /* PIOS_SENSOR_H */
