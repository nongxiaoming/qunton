#ifndef __DRV_BMP085_H
#define __DRV_BMP085_H

#include "rtos.h"


//! The valid oversampling rates
enum bmp085_osr {
    BMP085_OSR_0  = 0,
    BMP085_OSR_1  = 1,
    BMP085_OSR_2  = 2,
    BMP085_OSR_3  = 3,
};

//! Configuration structure for the BMP085 driver
struct bmp085_cfg {
    //! The oversampling setting for the baro, higher produces
    //! less frequenct cleaner data
    enum bmp085_osr oversampling;

    //! How many samples of pressure for each temperature measurement
    uint32_t temperature_interleaving;
};



/* Public Functions */
extern  int32_t BMP085_Init(struct bmp085_cfg *cfg, int32_t i2c_device);
extern int32_t BMP085_Test(void);

#endif /* __DRV_BMP085_H */

/** 
  * @}
  * @}
  */
