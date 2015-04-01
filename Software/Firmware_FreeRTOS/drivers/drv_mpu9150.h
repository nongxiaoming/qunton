#ifndef __DRV_MPU9150_H
#define __DRV_MPU9150_H

#include "rtos.h"
#include "drv_mpu6000.h"

/* MPU9150 I2C Addresses */
#define MPU9150_I2C_ADD_A0_LOW       0x68
#define MPU9150_I2C_ADD_A0_HIGH      0x69

/* Public Functions */
extern int32_t MPU9150_Init(uint32_t i2c_id, uint8_t i2c_addr, const struct mpu60x0_cfg * new_cfg);
extern uint8_t MPU9150_Test();
extern int32_t MPU9150_Probe(uint32_t i2c_id, uint8_t i2c_addr);
extern int32_t MPU9150_SetGyroRange(enum mpu60x0_range);
extern int32_t MPU9150_SetAccelRange(enum mpu60x0_accel_range);
extern int32_t MPU9150_SetSampleRate(uint16_t samplerate_hz);
extern void MPU9150_SetLPF(enum mpu60x0_filter filter);
extern bool_t MPU9150_IRQHandler(void);

#endif /* PIOS_MPU9150_H */

/** 
  * @}
  * @}
  */
