#ifndef __DRV_MPU6000_H
#define __DRV_MPU6000_H
#include "rtos.h"

/* MPU60X0 Addresses */
#define MPU60X0_SMPLRT_DIV_REG       0X19
#define MPU60X0_DLPF_CFG_REG         0X1A
#define MPU60X0_GYRO_CFG_REG         0X1B
#define MPU60X0_ACCEL_CFG_REG        0X1C
#define MPU60X0_FIFO_EN_REG          0x23
#define MPU60X0_SLV0_ADDR_REG        0x25
#define MPU60X0_SLV0_REG_REG         0x26
#define MPU60X0_SLV0_CTRL_REG        0x27
#define MPU60X0_SLV4_ADDR_REG        0x31
#define MPU60X0_SLV4_REG_REG         0x32
#define MPU60X0_SLV4_DO_REG          0x33
#define MPU60X0_SLV4_CTRL_REG        0x34
#define MPU60X0_SLV4_DI_REG          0x35
#define MPU60X0_I2C_MST_STATUS_REG   0x36
#define MPU60X0_INT_CFG_REG          0x37
#define MPU60X0_INT_EN_REG           0x38
#define MPU60X0_INT_STATUS_REG       0x3A
#define MPU60X0_ACCEL_X_OUT_MSB      0x3B
#define MPU60X0_ACCEL_X_OUT_LSB      0x3C
#define MPU60X0_ACCEL_Y_OUT_MSB      0x3D
#define MPU60X0_ACCEL_Y_OUT_LSB      0x3E
#define MPU60X0_ACCEL_Z_OUT_MSB      0x3F
#define MPU60X0_ACCEL_Z_OUT_LSB      0x40
#define MPU60X0_TEMP_OUT_MSB         0x41
#define MPU60X0_TEMP_OUT_LSB         0x42
#define MPU60X0_GYRO_X_OUT_MSB       0x43
#define MPU60X0_GYRO_X_OUT_LSB       0x44
#define MPU60X0_GYRO_Y_OUT_MSB       0x45
#define MPU60X0_GYRO_Y_OUT_LSB       0x46
#define MPU60X0_GYRO_Z_OUT_MSB       0x47
#define MPU60X0_GYRO_Z_OUT_LSB       0x48
#define MPU60X0_SIGNAL_PATH_RESET    0x68
#define MPU60X0_USER_CTRL_REG        0x6A
#define MPU60X0_PWR_MGMT_REG         0x6B
#define MPU60X0_FIFO_CNT_MSB         0x72
#define MPU60X0_FIFO_CNT_LSB         0x73
#define MPU60X0_FIFO_REG             0x74
#define MPU60X0_WHOAMI               0x75

/* FIFO enable for storing different values */
#define MPU60X0_FIFO_TEMP_OUT        0x80
#define MPU60X0_FIFO_GYRO_X_OUT      0x40
#define MPU60X0_FIFO_GYRO_Y_OUT      0x20
#define MPU60X0_FIFO_GYRO_Z_OUT      0x10
#define MPU60X0_ACCEL_OUT            0x08

/* Interrupt Configuration */
#define MPU60X0_INT_ACTL             0x80
#define MPU60X0_INT_OPEN             0x40
#define MPU60X0_INT_LATCH_EN         0x20
#define MPU60X0_INT_CLR_ANYRD        0x10

#define MPU60X0_INTEN_OVERFLOW       0x10
#define MPU60X0_INTEN_DATA_RDY       0x01

/* Interrupt status */
#define MPU60X0_INT_STATUS_OVERFLOW  0x10
#define MPU60X0_INT_STATUS_IMU_RDY   0X04
#define MPU60X0_INT_STATUS_DATA_RDY  0X01

/* User control functionality */
#define MPU60X0_USERCTL_FIFO_EN      0X40
#define MPU60X0_USERCTL_I2C_MST_EN   0X20
#define MPU60X0_USERCTL_DIS_I2C      0X10
#define MPU60X0_USERCTL_FIFO_RST     0X02
#define MPU60X0_USERCTL_GYRO_RST     0X01

/* Power management and clock selection */
#define MPU60X0_PWRMGMT_IMU_RST      0X80
#define MPU60X0_PWRMGMT_INTERN_CLK   0X00
#define MPU60X0_PWRMGMT_PLL_X_CLK    0X01
#define MPU60X0_PWRMGMT_PLL_Y_CLK    0X02
#define MPU60X0_PWRMGMT_PLL_Z_CLK    0X03
#define MPU60X0_PWRMGMT_STOP_CLK     0X07

/* I2C master status register bits */
#define MPU60X0_I2C_MST_SLV4_DONE    0x40
#define MPU60X0_I2C_MST_LOST_ARB     0x20
#define MPU60X0_I2C_MST_SLV4_NACK    0x10
#define MPU60X0_I2C_MST_SLV0_NACK    0x01

/* I2C SLV register bits */
#define MPU60X0_I2CSLV_EN            0x80
#define MPU60X0_I2CSLV_BYTE_SW       0x40
#define MPU60X0_I2CSLV_REG_DIS       0x20
#define MPU60X0_I2CSLV_GRP           0x10

enum mpu60x0_range {
	MPU60X0_SCALE_250_DEG  = 0x00,
	MPU60X0_SCALE_500_DEG  = 0x08,
	MPU60X0_SCALE_1000_DEG = 0x10,
	MPU60X0_SCALE_2000_DEG = 0x18
};

enum mpu60x0_filter {
	MPU60X0_LOWPASS_256_HZ = 0x00,
	MPU60X0_LOWPASS_188_HZ = 0x01,
	MPU60X0_LOWPASS_98_HZ  = 0x02,
	MPU60X0_LOWPASS_42_HZ  = 0x03,
	MPU60X0_LOWPASS_20_HZ  = 0x04,
	MPU60X0_LOWPASS_10_HZ  = 0x05,
	MPU60X0_LOWPASS_5_HZ   = 0x06
};

enum mpu60x0_accel_range {
	MPU60X0_ACCEL_2G = 0x00,
	MPU60X0_ACCEL_4G = 0x08,
	MPU60X0_ACCEL_8G = 0x10,
	MPU60X0_ACCEL_16G = 0x18
};

enum mpu60x0_orientation { // clockwise rotation from board forward
	MPU60X0_TOP_0DEG    = 0x00,
	MPU60X0_TOP_90DEG   = 0x01,
	MPU60X0_TOP_180DEG  = 0x02,
	MPU60X0_TOP_270DEG  = 0x03,
	MPU60X0_BOTTOM_0DEG  = 0x04,
	MPU60X0_BOTTOM_90DEG  = 0x05,
	MPU60X0_BOTTOM_180DEG  = 0x06,
	MPU60X0_BOTTOM_270DEG  = 0x07,
};

struct mpu60x0_data {
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;

	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t temperature;
};

struct mpu60x0_cfg {
	struct exti_cfg *exti_cfg; /* Pointer to the EXTI configuration */

	uint16_t default_samplerate;	/* Sample to use in Hz (See datasheet page 32 for more details) */
	uint8_t interrupt_cfg;			/* Interrupt configuration (See datasheet page 35 for more details) */
	uint8_t interrupt_en;			/* Interrupt configuration (See datasheet page 35 for more details) */
	uint8_t User_ctl;				/* User control settings (See datasheet page 41 for more details)  */
	uint8_t Pwr_mgmt_clk;			/* Power management and clock selection (See datasheet page 32 for more details) */
	enum mpu60x0_filter default_filter;
	enum mpu60x0_orientation orientation;
	uint8_t use_internal_mag;		/* Flag to indicate whether or not to use the internal mag on MPU9x50 devices */
};

/* Public Functions */
int32_t MPU6000_Init(uint32_t spi_id, uint32_t slave_num, const struct mpu60x0_cfg *new_cfg);
int32_t MPU6000_Test(void);
void MPU6000_SetGyroRange(enum mpu60x0_range);


void MPU6000_SetAccelRange(enum mpu60x0_accel_range);


void MPU6000_SetSampleRate(uint16_t samplerate_hz);
void MPU6000_SetLPF(enum mpu60x0_filter filter);
bool_t MPU6000_IRQHandler(void);

#endif /* __DRV_MPU6000_H */

/** 
  * @}
  * @}
  */
