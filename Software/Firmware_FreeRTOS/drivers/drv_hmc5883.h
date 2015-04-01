#ifndef __DRV_HMC5883_H
#define __DRV_HMC5883_H
#include "drv_i2c.h"

/* HMC5883 Addresses */
#define HMC5883_I2C_ADDR			        0x1E
#define HMC5883_I2C_READ_ADDR         0x3D
#define HMC5883_I2C_WRITE_ADDR        0x3C
#define HMC5883_CONFIG_REG_A		      0x00
#define HMC5883_CONFIG_REG_B		      0x01
#define HMC5883_MODE_REG			        0x02
#define HMC5883_DATAOUT_XMSB_REG		  0x03
#define HMC5883_DATAOUT_XLSB_REG		  0x04
#define HMC5883_DATAOUT_ZMSB_REG		  0x05
#define HMC5883_DATAOUT_ZLSB_REG		  0x06
#define HMC5883_DATAOUT_YMSB_REG		  0x07
#define HMC5883_DATAOUT_YLSB_REG		  0x08
#define HMC5883_DATAOUT_STATUS_REG		0x09
#define HMC5883_DATAOUT_IDA_REG		    0x0A
#define HMC5883_DATAOUT_IDB_REG		    0x0B
#define HMC5883_DATAOUT_IDC_REG		    0x0C

/* Output Data Rate */
#define HMC5883_ODR_0_75		          0x00
#define HMC5883_ODR_1_5		            0x04
#define HMC5883_ODR_3			            0x08
#define HMC5883_ODR_7_5		            0x0C
#define HMC5883_ODR_15			          0x10
#define HMC5883_ODR_30			          0x14
#define HMC5883_ODR_75			          0x18

/* Measure configuration */
#define HMC5883_MEASCONF_NORMAL		    0x00
#define HMC5883_MEASCONF_BIAS_POS		  0x01
#define HMC5883_MEASCONF_BIAS_NEG		  0x02

/* Gain settings */
#define HMC5883_GAIN_0_88			        0x00
#define HMC5883_GAIN_1_3			        0x20
#define HMC5883_GAIN_1_9			        0x40
#define HMC5883_GAIN_2_5			        0x60
#define HMC5883_GAIN_4_0			        0x80
#define HMC5883_GAIN_4_7			        0xA0
#define HMC5883_GAIN_5_6			        0xC0
#define HMC5883_GAIN_8_1			        0xE0

/* Modes */
#define HMC5883_MODE_CONTINUOUS	      0x00
#define HMC5883_MODE_SINGLE		        0x01
#define HMC5883_MODE_IDLE			        0x02
#define HMC5883_MODE_SLEEP			      0x03

/* Sensitivity Conversion Values */
enum pios_hmc5883_sensitivity {
	HMC5883_Sensitivity_0_88Ga = 1370, // LSB/Ga
	HMC5883_Sensitivity_1_3Ga  = 1090, // LSB/Ga
	HMC5883_Sensitivity_1_9Ga  = 820, // LSB/Ga
	HMC5883_Sensitivity_2_5Ga  = 660, // LSB/Ga
	HMC5883_Sensitivity_4_0Ga  = 440, // LSB/Ga
	HMC5883_Sensitivity_4_7Ga  = 390, // LSB/Ga
	HMC5883_Sensitivity_5_6Ga  = 330, // LSB/Ga
	HMC5883_Sensitivity_8_1Ga  = 230, // LSB/Ga  --> NOT RECOMMENDED
};

enum hmc5883_orientation {
	// clockwise rotation from board forward while looking at top side
	// 0 degree is chip mark on upper left corner
	HMC5883_TOP_0DEG,
	HMC5883_TOP_90DEG,
	HMC5883_TOP_180DEG,
	HMC5883_TOP_270DEG,
	// clockwise rotation from board forward while looking at bottom side
	// 0 degree is chip mark on upper left corner
	HMC5883_BOTTOM_0DEG,
	HMC5883_BOTTOM_90DEG,
	HMC5883_BOTTOM_180DEG,
	HMC5883_BOTTOM_270DEG
};

struct hmc5883_cfg {
	struct exti_cfg * exti_cfg; /* Pointer to the EXTI configuration */
	uint8_t M_ODR;		/* OUTPUT DATA RATE --> here below the relative define (See datasheet page 11 for more details) */
	uint8_t Meas_Conf;	/* Measurement Configuration,: Normal, positive bias, or negative bias --> here below the relative define */
	uint8_t Gain;		/* Gain Configuration, select the full scale --> here below the relative define (See datasheet page 11 for more details) */
	uint8_t Mode;
	enum hmc5883_orientation Default_Orientation;
};

struct hmc5883_data {
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
};

/* Public Functions */
extern int32_t HMC5883_Init(struct hmc5883_cfg * cfg);
extern int32_t HMC5883_Test(void);
extern int32_t HMC5883_SetOrientation(enum hmc5883_orientation orientation);
extern portBASE_TYPE HMC5883_IRQHandler(void);
#endif /* PIOS_HMC5883_H */

/** 
  * @}
  * @}
  */
