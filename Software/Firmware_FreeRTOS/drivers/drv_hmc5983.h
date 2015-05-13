#ifndef __DRV_HMC5983_H
#define __DRV_HMC5983_H

#include "rtos.h"

/* HMC5983 Addresses */
#define HMC5983_I2C_ADDR			0x1E
#define HMC5983_I2C_READ_ADDR		0x3D
#define HMC5983_I2C_WRITE_ADDR		0x3C
#define HMC5983_CONFIG_REG_A		(uint8_t)0x00
#define HMC5983_CONFIG_REG_B		(uint8_t)0x01
#define HMC5983_MODE_REG			(uint8_t)0x02
#define HMC5983_DATAOUT_XMSB_REG		0x03
#define HMC5983_DATAOUT_XLSB_REG		0x04
#define HMC5983_DATAOUT_ZMSB_REG		0x05
#define HMC5983_DATAOUT_ZLSB_REG		0x06
#define HMC5983_DATAOUT_YMSB_REG		0x07
#define HMC5983_DATAOUT_YLSB_REG		0x08
#define HMC5983_DATAOUT_STATUS_REG		0x09
#define HMC5983_DATAOUT_IDA_REG		0x0A
#define HMC5983_DATAOUT_IDB_REG		0x0B
#define HMC5983_DATAOUT_IDC_REG		0x0C
#define HMC5983_DATAOUT_TEMPMSB_REG	0x31
#define HMC5983_DATAOUT_TEMPLSB_REG	0x32

/* Output Data Rate */
#define HMC5983_ODR_0_75		0x00
#define HMC5983_ODR_1_5		0x04
#define HMC5983_ODR_3			0x08
#define HMC5983_ODR_7_5		0x0C
#define HMC5983_ODR_15			0x10
#define HMC5983_ODR_30			0x14
#define HMC5983_ODR_75			0x18
#define HMC5983_ODR_220		0x1c

/* Measure configuration */
#define HMC5983_MEASCONF_NORMAL		0x00
#define HMC5983_MEASCONF_BIAS_POS		0x01
#define HMC5983_MEASCONF_BIAS_NEG		0x02

/* Gain settings */
#define HMC5983_GAIN_0_88			0x00
#define HMC5983_GAIN_1_3			0x20
#define HMC5983_GAIN_1_9			0x40
#define HMC5983_GAIN_2_5			0x60
#define HMC5983_GAIN_4_0			0x80
#define HMC5983_GAIN_4_7			0xA0
#define HMC5983_GAIN_5_6			0xC0
#define HMC5983_GAIN_8_1			0xE0

/* Modes */
#define HMC5983_MODE_CONTINUOUS	0x00
#define HMC5983_MODE_SINGLE		0x01
#define HMC5983_MODE_IDLE			0x02
#define HMC5983_MODE_SLEEP			0x03

/* Sensitivity Conversion Values */
#define HMC5983_Sensitivity_0_88Ga		1370	// LSB/Ga
#define HMC5983_Sensitivity_1_3Ga		1090	// LSB/Ga
#define HMC5983_Sensitivity_1_9Ga		820	// LSB/Ga
#define HMC5983_Sensitivity_2_5Ga		660	// LSB/Ga
#define HMC5983_Sensitivity_4_0Ga		440	// LSB/Ga
#define HMC5983_Sensitivity_4_7Ga		390	// LSB/Ga
#define HMC5983_Sensitivity_5_6Ga		330	// LSB/Ga
#define HMC5983_Sensitivity_8_1Ga		230	// LSB/Ga  --> NOT RECOMMENDED

#define HMC5983_AVERAGING_1			0x00
#define HMC5983_AVERAGING_2			0x20
#define HMC5983_AVERAGING_4			0x40
#define HMC5983_AVERAGING_8			0x60

#define HMC5983_READ_MODE				0xC0
#define HMC5983_ENABLE_TEMP_SENSOR		0x80

enum hmc5983_orientation {
	// clockwise rotation from board forward while looking at top side
	// 0 degree is chip mark on upper left corner
	HMC5983_TOP_0DEG,
	HMC5983_TOP_90DEG,
	HMC5983_TOP_180DEG,
	HMC5983_TOP_270DEG,
	// clockwise rotation from board forward while looking at bottom side
	// 0 degree is chip mark on upper left corner
	HMC5983_BOTTOM_0DEG,
	HMC5983_BOTTOM_90DEG,
	HMC5983_BOTTOM_180DEG,
	HMC5983_BOTTOM_270DEG
};

struct hmc5983_cfg {
	const struct exti_cfg * exti_cfg; /* Pointer to the EXTI configuration */
	uint8_t M_ODR;		/* OUTPUT DATA RATE --> here below the relative define (See datasheet page 11 for more details) */
	uint8_t Meas_Conf;	/* Measurement Configuration,: Normal, positive bias, or negative bias --> here below the relative define */
	uint8_t Gain;		/* Gain Configuration, select the full scale --> here below the relative define (See datasheet page 11 for more details) */
	uint8_t Averaging;	/* Averaging configuration */
	uint8_t Mode;
	enum hmc5983_orientation Orientation;
};

/* Public Functions */
extern int32_t HMC5983_Init(uint32_t spi_i2c_id, uint32_t slave_num, struct hmc5983_cfg *cfg);
extern bool_t HMC5983_IRQHandler(void);

extern int32_t HMC5983_Test(void);
extern int32_t HMC5983_SetOrientation(enum hmc5983_orientation orientation);


#endif /* __DRV_HMC5983_H */

/**
  * @}
  * @}
  */
