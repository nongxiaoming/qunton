#ifndef __DRV_L3GD20_H
#define __DRV_L3GD20_H
#include "rtos.h"


/* L3GD20 Addresses */
#define L3GD20_WHOAMI               0x0F
#define L3GD20_CTRL_REG1            0X20
#define L3GD20_CTRL_REG2            0X21
#define L3GD20_CTRL_REG3            0X22
#define L3GD20_CTRL_REG4            0X23
#define L3GD20_CTRL_REG5            0X24
#define L3GD20_REFERENCE            0X25
#define L3GD20_OUT_TEMP             0x26
#define L3GD20_STATUS_REG           0x27
#define L3GD20_GYRO_X_OUT_LSB       0x28
#define L3GD20_GYRO_X_OUT_MSB       0x29
#define L3GD20_GYRO_Y_OUT_LSB       0x2A
#define L3GD20_GYRO_Y_OUT_MSB       0x2B
#define L3GD20_GYRO_Z_OUT_LSB       0x2C
#define L3GD20_GYRO_Z_OUT_MSB       0x2D
#define L3GD20_FIFO_CTRL_REG        0x2E
#define L3GD20_FIFO_SRC_REG         0x2F
#define L3GD20_INT1_CFG             0x30
#define L3GD20_INT1_SRC             0x31
#define L3GD20_INT1_TSH_XH          0x32
#define L3GD20_INT1_TSH_XL          0x33
#define L3GD20_INT1_TSH_YH          0x34
#define L3GD20_INT1_TSH_YL          0x35
#define L3GD20_INT1_TSH_ZH          0x36
#define L3GD20_INT1_TSH_ZL          0x37
#define L3GD20_INT1_DURATION        0x38

/* Ctrl1 flags */
#define L3GD20_CTRL1_PD             0x08
#define L3GD20_CTRL1_ZEN            0x04
#define L3GD20_CTRL1_YEN            0x02
#define L3GD20_CTRL1_XEN            0x01

/* FIFO enable for storing different values */
#define L3GD20_FIFO_TEMP_OUT        0x80
#define L3GD20_FIFO_GYRO_X_OUT      0x40
#define L3GD20_FIFO_GYRO_Y_OUT      0x20
#define L3GD20_FIFO_GYRO_Z_OUT      0x10
#define L3GD20_ACCEL_OUT            0x08

/* Interrupt Configuration */
#define L3GD20_INT_ACTL             0x80
#define L3GD20_INT_OPEN             0x40
#define L3GD20_INT_LATCH_EN         0x20
#define L3GD20_INT_CLR_ANYRD        0x10

#define L3GD20_INTEN_OVERFLOW       0x10
#define L3GD20_INTEN_DATA_RDY       0x01

/* Interrupt status */
#define L3GD20_INT_STATUS_FIFO_FULL 0x80
#define L3GD20_INT_STATUS_IMU_RDY   0X04
#define L3GD20_INT_STATUS_DATA_RDY  0X01

/* User control functionality */
#define L3GD20_USERCTL_FIFO_EN      0X40
#define L3GD20_USERCTL_FIFO_RST     0X02
#define L3GD20_USERCTL_GYRO_RST     0X01

/* Power management and clock selection */
#define L3GD20_PWRMGMT_IMU_RST      0X80
#define L3GD20_PWRMGMT_INTERN_CLK   0X00
#define L3GD20_PWRMGMT_PLL_X_CLK    0X01
#define L3GD20_PWRMGMT_PLL_Y_CLK    0X02
#define L3GD20_PWRMGMT_PLL_Z_CLK    0X03
#define L3GD20_PWRMGMT_STOP_CLK     0X07

enum l3gd20_range {
	L3GD20_SCALE_250_DEG  = 0x00,
	L3GD20_SCALE_500_DEG  = 0x10,
	L3GD20_SCALE_2000_DEG = 0x3
};

enum l3gd20_rate {
	L3GD20_RATE_760HZ_100HZ = 0xF0,
	L3GD20_RATE_380HZ_100HZ = 0xB0
};

enum l3gd20_filter {
	L3GD20_LOWPASS_256_HZ = 0x00,
	L3GD20_LOWPASS_188_HZ = 0x01,
	L3GD20_LOWPASS_98_HZ  = 0x02,
	L3GD20_LOWPASS_42_HZ  = 0x03,
	L3GD20_LOWPASS_20_HZ  = 0x04,
	L3GD20_LOWPASS_10_HZ  = 0x05,
	L3GD20_LOWPASS_5_HZ   = 0x06
};


struct l3gd20_cfg {
	struct exti_cfg * exti_cfg; /* Pointer to the EXTI configuration */

	enum l3gd20_range range;
};

/* Public Functions */
int32_t L3GD20_Init(uint32_t spi_id, uint32_t slave_num, struct l3gd20_cfg * cfg);
int32_t L3GD20_SetRange(enum l3gd20_range range);
int32_t L3GD20_SetSampleRate(enum l3gd20_rate rate);
int32_t L3GD20_ReadID(void);
uint8_t L3GD20_Test(void);
bool_t L3GD20_IRQHandler(void);

#endif /* __DRV_L3GD20_H */

/** 
  * @}
  * @}
  */
