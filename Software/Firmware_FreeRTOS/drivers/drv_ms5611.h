#ifndef __DRV_MS5611_H
#define __DRV_MS5611_H
#include "drv_i2c.h"
//! The valid oversampling rates
enum ms5611_osr {
	MS5611_OSR_256   = 0,
	MS5611_OSR_512   = 2,
	MS5611_OSR_1024  = 4,
	MS5611_OSR_2048  = 6,
	MS5611_OSR_4096  = 8,
};

//! Configuration structure for the MS5611 driver
struct ms5611_cfg {
	//! The oversampling setting for the baro, higher produces
	//! less frequenct cleaner data
	enum ms5611_osr oversampling;

	//! How many samples of pressure for each temperature measurement
	uint32_t temperature_interleaving;
};

int32_t MS5611_Init(struct ms5611_cfg * cfg, int32_t i2c_device);


#endif /* PIOS_MS5611_PRIV_H */

/** 
  * @}
  * @}
  */
