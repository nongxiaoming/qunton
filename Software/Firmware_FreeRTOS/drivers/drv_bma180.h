#ifndef __DRV_BMA180_H
#define __DRV_BMA180_H

#include "rtos.h"

/* BMA180 Addresses */
#define BMA_CHIPID_ADDR   0x00
#define BMA_VERSION_ADDR  0x00
#define BMA_X_LSB_ADDR    0x02
#define BMA_Y_LSB_ADDR    0x04
#define BMA_Z_LSB_ADDR    0x06
#define BMA_WE_ADDR       0x0D
#define BMA_RESET         0x10
#define BMA_BW_ADDR       0x20
#define BMA_RANGE_ADDR    0x35
#define BMA_OFFSET_LSB1   0x35
#define BMA_GAIN_Y        0x33
#define BMA_CTRREG3       0x21
#define BMA_CTRREG0       0x0D

#define BMA_RESET_CODE    0x6B

/* Accel range  */
#define BMA_RANGE_MASK    0x0E          
#define BMA_RANGE_SHIFT   1
enum bma180_range { BMA_RANGE_1G = 0x00,
	BMA_RANGE_1_5G = 0x01,
	BMA_RANGE_2G = 0x02,
	BMA_RANGE_3G = 0x03,
	BMA_RANGE_4G = 0x04,
	BMA_RANGE_8G = 0x05,
	BMA_RANGE_16G = 0x06
};

/* Measurement bandwidth */
#define BMA_BW_MASK       0xF0
#define BMA_BW_SHIFT      4
enum bma180_bandwidth { BMA_BW_10HZ = 0x00,
	BMA_BW_20HZ = 0x01,
	BMA_BW_40HZ = 0x02,
	BMA_BW_75HZ = 0x03,
	BMA_BW_150HZ = 0x04,
	BMA_BW_300HZ = 0x05,
	BMA_BW_600HZ = 0x06,
	BMA_BW_1200HZ =0x07,
	BMA_BW_HP1HZ = 0x08,    // High-pass, 1 Hz
	BMA_BW_BP0_300HZ = 0x09 // Band-pass, 0.3Hz-300Hz
};

#define BMA_NEW_DAT_INT   0x02

struct bma180_data {
	int16_t x;
	int16_t y;
	int16_t z;
	int8_t temperature;
};


struct pios_bma180_cfg {
	const struct exti_cfg * exti_cfg; /* Pointer to the EXTI configuration */
	enum bma180_bandwidth bandwidth;
	enum bma180_range range;
};

/* Public Functions */
int32_t BMA180_Init(uint32_t spi_id, uint32_t slave_num, const struct bma180_cfg * cfg);
void    BMA180_Attach(uint32_t spi_id);
float   BMA180_GetScale();
int32_t BMA180_ReadFifo(struct bma180_data * buffer);
int32_t BMA180_ReadAccels(struct bma180_data * data);
int32_t BMA180_Test();
bool_t BMA180_IRQHandler();

#endif /* __DRV_BMA180_H */

/**
 * @}
 * @}
 */
