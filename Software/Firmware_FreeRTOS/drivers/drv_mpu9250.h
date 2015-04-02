#ifndef __DRV_MPU9250_H
#define __DRV_MPU9250_H
#include "rtos.h"

enum mpu9250_gyro_filter {
	MPU9250_GYRO_LOWPASS_250_HZ = 0x00,
	MPU9250_GYRO_LOWPASS_184_HZ = 0x01,
	MPU9250_GYRO_LOWPASS_92_HZ  = 0x02,
	MPU9250_GYRO_LOWPASS_41_HZ  = 0x03,
	MPU9250_GYRO_LOWPASS_20_HZ  = 0x04,
	MPU9250_GYRO_LOWPASS_10_HZ  = 0x05,
	MPU9250_GYRO_LOWPASS_5_HZ   = 0x06
};

enum mpu9250_accel_filter {
	MPU9250_ACCEL_LOWPASS_460_HZ = 0x00,
	MPU9250_ACCEL_LOWPASS_184_HZ = 0x01,
	MPU9250_ACCEL_LOWPASS_92_HZ  = 0x02,
	MPU9250_ACCEL_LOWPASS_41_HZ  = 0x03,
	MPU9250_ACCEL_LOWPASS_20_HZ  = 0x04,
	MPU9250_ACCEL_LOWPASS_10_HZ  = 0x05,
	MPU9250_ACCEL_LOWPASS_5_HZ   = 0x06
};

enum mpu9250_orientation { // clockwise rotation from board forward
	MPU9250_TOP_0DEG       = 0x00,
	MPU9250_TOP_90DEG      = 0x01,
	MPU9250_TOP_180DEG     = 0x02,
	MPU9250_TOP_270DEG     = 0x03,
	MPU9250_BOTTOM_0DEG    = 0x04,
	MPU9250_BOTTOM_90DEG   = 0x05,
	MPU9250_BOTTOM_180DEG  = 0x06,
	MPU9250_BOTTOM_270DEG  = 0x07
};


struct mpu9250_cfg {
	const struct pios_exti_cfg *exti_cfg; /* Pointer to the EXTI configuration */

	uint16_t default_samplerate;	/* Sample to use in Hz (See RM datasheet page 12 for more details) */
	uint8_t interrupt_cfg;			/* Interrupt configuration (See RM datasheet page 20 for more details) */
	bool use_magnetometer;			/* Use magnetometer or not - for example when external mag. is used */
	enum mpu9250_gyro_filter default_gyro_filter;
	enum mpu9250_accel_filter default_accel_filter;
	enum mpu9250_orientation orientation;
};

/* Public Functions */
extern int32_t MPU9250_SPI_Init(uint32_t spi_id, uint32_t slave_num, const struct mpu9250_cfg *new_cfg);
extern int32_t MPU9250_Test();
extern int32_t MPU9250_SetGyroRange(enum mpu60x0_range range);
extern int32_t MPU9250_SetAccelRange(enum mpu60x0_accel_range);
extern int32_t MPU9250_SetSampleRate(uint16_t samplerate_hz);
extern void MPU9250_SetGyroLPF(enum mpu9250_gyro_filter filter);
extern void MPU9250_SetAccelLPF(enum mpu9250_accel_filter filter);
extern bool MPU9250_IRQHandler(void);

#endif /* __DRV_MPU9250_H */

/** 
  * @}
  * @}
  */
