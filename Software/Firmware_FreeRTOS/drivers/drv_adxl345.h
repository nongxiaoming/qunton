#ifndef __DRV_ADXL345_H
#define __DRV_ADXL345_H
#include "rtos.h"
// Defined by data rate, not BW

#define ADXL_READ_BIT      0x80
#define ADXL_MULTI_BIT     0x40

#define ADXL_WHOAMI        0x00
#define ADXL_DEVICE_ID     0xE5
#define ADXL_X0_ADDR       0x32
#define ADXL_FIFOSTATUS_ADDR 0x39

#define ADXL_RATE_ADDR     0x2C
#define ADXL_RATE_100      0x0A
#define ADXL_RATE_200      0x0B
#define ADXL_RATE_400      0x0C
#define ADXL_RATE_800      0x0D
#define ADXL_RATE_1600     0x0E
#define ADXL_RATE_3200     0x0F

#define ADXL_POWER_ADDR    0x2D
#define ADXL_MEAURE        0x08

#define ADXL_FORMAT_ADDR   0x31
#define ADXL_FULL_RES      0x08
#define ADXL_4WIRE         0x00
#define ADXL_RANGE_2G      0x00
#define ADXL_RANGE_4G      0x01
#define ADXL_RANGE_8G      0x02
#define ADXL_RANGE_16G     0x03

#define ADXL_FIFO_ADDR     0x38
#define ADXL_FIFO_STREAM   0x80


struct adxl345_data {
	int16_t x;
	int16_t y;
	int16_t z;
};

int32_t ADXL345_SelectRate(uint8_t rate); 
int32_t ADXL345_SetRange(uint8_t range);
int32_t ADXL345_Init(uint32_t spi_id, uint32_t slave_num);
uint8_t ADXL345_Read(struct adxl345_data * data);
int32_t ADXL345_FifoElements(void);
int32_t ADXL345_Test(void);

#endif

/** 
 * @}
 * @}
 */
