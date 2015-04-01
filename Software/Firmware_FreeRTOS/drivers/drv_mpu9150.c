#include "drv_mpu9150.h"
#include "drv_mpu6000.h"


/* Private constants */
#define MPU9150_TASK_PRIORITY		THREAD_PRIO_HIGHEST
#define MPU9150_TASK_STACK_BYTES	512

#define MPU9150_WHOAMI_ID        0x68
#define MPU9150_MAG_ADDR         0x0c
#define MPU9150_MAG_STATUS       0x02
#define MPU9150_MAG_XH           0x03
#define MPU9150_MAG_XL           0x04
#define MPU9150_MAG_YH           0x05
#define MPU9150_MAG_YL           0x06
#define MPU9150_MAG_ZH           0x07
#define MPU9150_MAG_ZL           0x08
#define MPU9150_MAG_STATUS2      0x09
#define MPU9150_MAG_CNTR         0x0a

/* Global Variables */

#define MPU9150_MAX_DOWNSAMPLE 2
struct mpu9150_dev {
	uint32_t i2c_id;
	uint8_t i2c_addr;
	enum mpu60x0_accel_range accel_range;
	enum mpu60x0_range gyro_range;
	os_queue_t gyro_queue;
	os_queue_t accel_queue;
	os_queue_t mag_queue;
	os_task_t TaskHandle;
	os_sema_t data_ready_sema;
	struct mpu60x0_cfg * cfg;
	enum mpu60x0_filter filter;
};

//! Global structure for this device device
static struct mpu9150_dev * dev;

//! Private functions
static struct mpu9150_dev * MPU9150_alloc(void);
static int32_t MPU9150_Validate(struct mpu9150_dev * dev);
static int32_t MPU9150_Config(struct mpu60x0_cfg * cfg);
static int32_t MPU9150_SetReg(uint8_t address, uint8_t buffer);
static int32_t MPU9150_GetReg(uint8_t address);
static int32_t MPU9150_ReadID();
static int32_t MPU9150_Mag_SetReg(uint8_t reg, uint8_t buffer);
static int32_t MPU9150_Mag_GetReg(uint8_t reg);
static void   MPU9150_Task(void *parameters);

/**
 * @brief Allocate a new device
 */
static struct mpu9150_dev * MPU9150_alloc(void)
{
	struct mpu9150_dev * mpu9150_dev;
	
	mpu9150_dev = (struct mpu9150_dev *)OS_Malloc(sizeof(*mpu9150_dev));
	if (!mpu9150_dev) return (NULL);
	
	mpu9150_dev->accel_queue = OS_QueueCreate(MPU9150_MAX_DOWNSAMPLE, sizeof(struct sensor_gyro_data));
	if (mpu9150_dev->accel_queue == NULL) {
		OS_Free(mpu9150_dev);
		return NULL;
	}

	mpu9150_dev->gyro_queue = OS_QueueCreate(MPU9150_MAX_DOWNSAMPLE, sizeof(struct sensor_gyro_data));
	if (mpu9150_dev->gyro_queue == NULL) {
		OS_Free(mpu9150_dev);
		return NULL;
	}

	mpu9150_dev->mag_queue = OS_QueueCreate(MPU9150_MAX_DOWNSAMPLE, sizeof(struct sensor_mag_data));
	if (mpu9150_dev->mag_queue == NULL) {
		OS_Free(mpu9150_dev);
		return NULL;
	}

	mpu9150_dev->data_ready_sema = Semaphore_Create();
	if (mpu9150_dev->data_ready_sema == NULL) {
		OS_Free(mpu9150_dev);
		return NULL;
	}

	return mpu9150_dev;
}

/**
 * @brief Validate the handle to the i2c device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t MPU9150_Validate(struct mpu9150_dev * dev)
{
	if (dev == NULL) 
		return -1;
	if (dev->i2c_id == 0)
		return -2;
	return 0;
}

/**
 * @brief Initialize the MPU9150 3-axis gyro sensor.
 * @return 0 for success, -1 for failure to allocate, -2 for failure to get irq
 */
int32_t MPU9150_Init(uint32_t i2c_id, uint8_t i2c_addr, struct mpu60x0_cfg * cfg)
{
	dev = MPU9150_alloc();
	if (dev == NULL)
		return -1;
	
	dev->i2c_id = i2c_id;
	dev->i2c_addr = i2c_addr;
	dev->cfg = cfg;

	/* Configure the MPU9150 Sensor */
	if (MPU9150_Config(cfg) != 0)
		return -2;

	/* Set up EXTI line */
	EXTI_Init(cfg->exti_cfg);

	// Wait 5 ms for data ready interrupt and make sure it happens
	// twice
	if ((Semaphore_Take(dev->data_ready_sema, 5) != true) ||
		(Semaphore_Take(dev->data_ready_sema, 5) != true)) {
		return -10;
	}

	dev->TaskHandle = OS_TaskCreate(
			MPU9150_Task, "mpu9150", MPU9150_TASK_STACK_BYTES, NULL, MPU9150_TASK_PRIORITY);
	OS_Assert(dev->TaskHandle != NULL);

	SENSORS_Register(SENSOR_ACCEL, dev->accel_queue);
	SENSORS_Register(SENSOR_GYRO, dev->gyro_queue);
	SENSORS_Register(SENSOR_MAG, dev->mag_queue);

	return 0;
}

/**
 * @brief Initialize the MPU9150 3-axis gyro sensor
 * \return none
 * \param[in] PIOS_MPU9150_ConfigTypeDef struct to be used to configure sensor.
*
*/
static int32_t MPU9150_Config(struct mpu60x0_cfg * cfg)
{
	// Reset chip
	if (MPU9150_SetReg(MPU60X0_PWR_MGMT_REG, MPU60X0_PWRMGMT_IMU_RST) != 0)
		return -1;

	// Give chip some time to initialize
	OS_Delay(50);
	WDG_Clear();

	//Power management configuration
	MPU9150_SetReg(MPU60X0_PWR_MGMT_REG, cfg->Pwr_mgmt_clk);

	// User control
	MPU9150_SetReg(MPU60X0_USER_CTRL_REG, cfg->User_ctl & ~0x40);

	// Digital low-pass filter and scale
	// set this before sample rate else sample rate calculation will fail
	MPU9150_SetLPF(cfg->default_filter);

	// Sample rate
	MPU9150_SetSampleRate(cfg->default_samplerate);

	// Set the gyro scale
	MPU9150_SetGyroRange(MPU60X0_SCALE_500_DEG);

	// Set the accel scale
	MPU9150_SetAccelRange(MPU60X0_ACCEL_8G);

	// Interrupt configuration
	MPU9150_SetReg(MPU60X0_INT_CFG_REG, cfg->interrupt_cfg | 0x02);

	// To enable access to the mag on auxillary i2c we must set bit 0x02 in register 0x37
	// and clear bit 0x40 in register 0x6a (MPU60X0_USER_CTRL_REG, default condition)
	
	// Disable mag first
	if (MPU9150_Mag_SetReg(MPU9150_MAG_CNTR, 0x00) != 0)
		return -1;
	OS_Delay(20);
	WDG_Clear();
	// Clear status registers
	MPU9150_Mag_GetReg(MPU9150_MAG_STATUS);
	MPU9150_Mag_GetReg(MPU9150_MAG_STATUS2);
	MPU9150_Mag_GetReg(MPU9150_MAG_XH);

	if (dev->cfg->use_internal_mag == true) {
		// Trigger first measurement
		if (MPU9150_Mag_SetReg(MPU9150_MAG_CNTR, 0x01) != 0)
			return -1;
	}

	// Interrupt enable
	MPU9150_SetReg(MPU60X0_INT_EN_REG, cfg->interrupt_en);

	return 0;
}

/**
 * Set the gyro range and store it locally for scaling
 */
int32_t MPU9150_SetGyroRange(enum mpu60x0_range gyro_range)
{
	if (MPU9150_SetReg(MPU60X0_GYRO_CFG_REG, gyro_range) != 0)
		return -1;

	switch(gyro_range) {
	case MPU60X0_SCALE_250_DEG:
		SENSORS_SetMaxGyro(250);
		break;
	case MPU60X0_SCALE_500_DEG:
		SENSORS_SetMaxGyro(500);
		break;
	case MPU60X0_SCALE_1000_DEG:
		SENSORS_SetMaxGyro(1000);
		break;
	case MPU60X0_SCALE_2000_DEG:
		SENSORS_SetMaxGyro(2000);
		break;
	}

	dev->gyro_range = gyro_range;
	return 0;
}

/**
 * Set the accel range and store it locally for scaling
 */
int32_t MPU9150_SetAccelRange(enum mpu60x0_accel_range accel_range)
{
	if (MPU9150_SetReg(MPU60X0_ACCEL_CFG_REG, accel_range) != 0)
		return -1;
	dev->accel_range = accel_range;
	return 0;
}

/**
 * Set the sample rate in Hz by determining the nearest divisor
 * @param[in] sample rate in Hz
 */
int32_t MPU9150_SetSampleRate(uint16_t samplerate_hz)
{
	uint16_t filter_frequency = 8000;

	if (dev->filter != MPU60X0_LOWPASS_256_HZ)
		filter_frequency = 1000;

	// limit samplerate to filter frequency
	if (samplerate_hz > filter_frequency)
		samplerate_hz = filter_frequency;

	// calculate divisor, round to nearest integeter
	int32_t divisor = (int32_t)(((float)filter_frequency / samplerate_hz) + 0.5f) - 1;

	// limit resulting divisor to register value range
	if (divisor < 0)
		divisor = 0;

	if (divisor > 0xff)
		divisor = 0xff;

	return MPU9150_SetReg(MPU60X0_SMPLRT_DIV_REG, (uint8_t)divisor);
}

/**
 * Configure the digital low-pass filter
 */
void MPU9150_SetLPF(enum mpu60x0_filter filter)
{
	MPU9150_SetReg(MPU60X0_DLPF_CFG_REG, filter);

	dev->filter = filter;
}

/**
 * Check if an MPU9150 is detected at the requested address
 * @return 0 if detected, -1 if successfully probed but wrong id
 *  -2 no device at address
 */
int32_t MPU9150_Probe(uint32_t i2c_id, uint8_t i2c_addr)
{
	// This function needs to set up the full transactions because
	// it should not assume anything is configured

	uint8_t mag_addr_buffer[] = {
		0,
	};
	uint8_t mag_read_buffer[] = {
		0
	};

	const struct i2c_txn mag_txn_list[] = {
		{
			.info = __func__,
			.addr = MPU9150_MAG_ADDR,
			.rw = I2C_TXN_WRITE,
			.len = sizeof(mag_addr_buffer),
			.buf = mag_addr_buffer,
		},
		{
			.info = __func__,
			.addr = MPU9150_MAG_ADDR,
			.rw = I2C_TXN_READ,
			.len = sizeof(mag_read_buffer),
			.buf = mag_read_buffer,
		}
	};

	int32_t retval = I2C_Transfer(i2c_id, mag_txn_list, NELEMENTS(mag_txn_list));
	if (retval < 0)
		return -1;

	return 0;
}

/**
 * @brief Reads one or more bytes into a buffer
 * \param[in] address MPU9150 register address (depends on size)
 * \param[out] buffer destination buffer
 * \param[in] len number of bytes which should be read
 * \return 0 if operation was successful
 * \return -1 if error during I2C transfer
 * \return -2 if unable to claim i2c device
 */
static int32_t MPU9150_Read(uint8_t address, uint8_t * buffer, uint8_t len)
{
	uint8_t addr_buffer[] = {
		address,
	};

	const struct i2c_txn txn_list[] = {
		{
			.info = __func__,
			.addr = dev->i2c_addr,
			.rw = I2C_TXN_WRITE,
			.len = sizeof(addr_buffer),
			.buf = addr_buffer,
		},
		{
			.info = __func__,
			.addr = dev->i2c_addr,
			.rw = I2C_TXN_READ,
			.len = len,
			.buf = buffer,
		}
	};

	return I2C_Transfer(dev->i2c_id, txn_list, NELEMENTS(txn_list));
}

/**
 * @brief Writes one or more bytes to the MPU9150
 * \param[in] address Register address
 * \param[in] buffer source buffer
 * \return 0 if operation was successful
 * \return -1 if error during I2C transfer
 * \return -2 if unable to claim i2c device
 */
static int32_t MPU9150_Write(uint8_t address, uint8_t buffer)
{
	uint8_t data[] = {
		address,
		buffer,
	};

	const struct i2c_txn txn_list[] = {
		{
			.info = __func__,
			.addr = dev->i2c_addr,
			.rw = I2C_TXN_WRITE,
			.len = sizeof(data),
			.buf = data,
		},
	};

	return I2C_Transfer(dev->i2c_id, txn_list, NELEMENTS(txn_list));
}

/**
 * @brief Read a register from MPU9150
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t MPU9150_GetReg(uint8_t reg)
{
	uint8_t data;

	int32_t retval = MPU9150_Read(reg, &data, sizeof(data));

	if (retval != 0)
		return retval;
	else
		return data;
}

/**
 * @brief Writes one byte to the MPU9150
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int32_t MPU9150_SetReg(uint8_t reg, uint8_t data)
{
	return MPU9150_Write(reg, data);
}

/**
 * @brief Reads one or more bytes into a buffer
 * \param[in] address MPU9150 register address (depends on size)
 * \param[out] buffer destination buffer
 * \param[in] len number of bytes which should be read
 * \return 0 if operation was successful
 * \return -1 if error during I2C transfer
 * \return -2 if unable to claim i2c device
 */
static int32_t MPU9150_Mag_Read(uint8_t address, uint8_t * buffer, uint8_t len)
{
	uint8_t addr_buffer[] = {
		address,
	};

	const struct i2c_txn txn_list[] = {
		{
			.info = __func__,
			.addr = MPU9150_MAG_ADDR,
			.rw = I2C_TXN_WRITE,
			.len = sizeof(addr_buffer),
			.buf = addr_buffer,
		},
		{
			.info = __func__,
			.addr = MPU9150_MAG_ADDR,
			.rw = I2C_TXN_READ,
			.len = len,
			.buf = buffer,
		}
	};

	return I2C_Transfer(dev->i2c_id, txn_list, NELEMENTS(txn_list));
}

/**
 * @brief Writes one or more bytes to the MPU9150
 * \param[in] address Register address
 * \param[in] buffer source buffer
 * \return 0 if operation was successful
 * \return -1 if error during I2C transfer
 * \return -2 if unable to claim i2c device
 */
static int32_t MPU9150_Mag_Write(uint8_t address, uint8_t buffer)
{
	uint8_t data[] = {
		address,
		buffer,
	};

	const struct i2c_txn txn_list[] = {
		{
			.info = __func__,
			.addr = MPU9150_MAG_ADDR,
			.rw = I2C_TXN_WRITE,
			.len = sizeof(data),
			.buf = data,
		},
	};

	return I2C_Transfer(dev->i2c_id, txn_list, NELEMENTS(txn_list));
}

/**
 * @brief Read a register from MPU9150
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t MPU9150_Mag_GetReg(uint8_t reg)
{
	uint8_t data;

	int32_t retval = MPU9150_Mag_Read(reg, &data, sizeof(data));

	if (retval != 0)
		return retval;
	else
		return data;
}

/**
 * @brief Writes one byte to the MPU9150
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int32_t MPU9150_Mag_SetReg(uint8_t reg, uint8_t data)
{
	return MPU9150_Mag_Write(reg, data);
}

/*
 * @brief Read the identification bytes from the MPU9150 sensor
 * \return ID read from MPU9150 or -1 if failure
*/
static int32_t MPU9150_ReadID()
{
	int32_t mpu9150_id = MPU9150_GetReg(MPU60X0_WHOAMI);
	if (mpu9150_id < 0)
		return -1;
	return mpu9150_id;
}


static float MPU9150_GetGyroScale()
{
	switch (dev->gyro_range) {
		case MPU60X0_SCALE_250_DEG:
			return 1.0f / 131.0f;
		case MPU60X0_SCALE_500_DEG:
			return 1.0f / 65.5f;
		case MPU60X0_SCALE_1000_DEG:
			return 1.0f / 32.8f;
		case MPU60X0_SCALE_2000_DEG:
			return 1.0f / 16.4f;
	}
	return 0;
}

#if defined(MPU6050_ACCEL)
static float MPU9150_GetAccelScale()
{
	switch (dev->accel_range) {
		case MPU60X0_ACCEL_2G:
			return GRAVITY / 16384.0f;
		case MPU60X0_ACCEL_4G:
			return GRAVITY / 8192.0f;
		case MPU60X0_ACCEL_8G:
			return GRAVITY / 4096.0f;
		case MPU60X0_ACCEL_16G:
			return GRAVITY / 2048.0f;
	}
	return 0;
}
#endif /* MPU6050_ACCEL */

//! Return mGa / LSB
static float MPU9150_GetMagScale()
{
	return 3.0f; //(1229.0*10.0/4096.0)
}

/**
 * @brief Run self-test operation.
 * \return 0 if test succeeded
 * \return non-zero value if test succeeded
 */
uint8_t MPU9150_Test(void)
{
	/* Verify that ID matches (MPU9150 ID is MPU9150_WHOAMI_ID) */
	int32_t mpu9150_id = MPU9150_ReadID();
	if (mpu9150_id < 0)
		return -1;
	
	if (mpu9150_id != MPU9150_WHOAMI_ID)
		return -2;
	
	return 0;
}

/**
* @brief IRQ Handler.  Read all the data from onboard buffer
*/
bool_t MPU9150_IRQHandler(void)
{
	if (MPU9150_Validate(dev) != 0)
		return FALSE;

	bool_t woken = FALSE;

	Semaphore_Give_FromISR(dev->data_ready_sema, &woken);

	return woken;
}

static void MPU9150_Task(void *parameters)
{
	while (1) {
		//Wait for data ready interrupt
		if (Semaphore_Take(dev->data_ready_sema, SEMAPHORE_TIMEOUT_MAX) != TRUE)
			continue;

		enum {
		    IDX_ACCEL_XOUT_H = 0,
		    IDX_ACCEL_XOUT_L,
		    IDX_ACCEL_YOUT_H,
		    IDX_ACCEL_YOUT_L,
		    IDX_ACCEL_ZOUT_H,
		    IDX_ACCEL_ZOUT_L,
		    IDX_TEMP_OUT_H,
		    IDX_TEMP_OUT_L,
		    IDX_GYRO_XOUT_H,
		    IDX_GYRO_XOUT_L,
		    IDX_GYRO_YOUT_H,
		    IDX_GYRO_YOUT_L,
		    IDX_GYRO_ZOUT_H,
		    IDX_GYRO_ZOUT_L,
		    BUFFER_SIZE,
		};


		uint8_t mpu9150_rec_buf[BUFFER_SIZE];

		if (MPU9150_Read(MPU60X0_ACCEL_X_OUT_MSB, mpu9150_rec_buf, sizeof(mpu9150_rec_buf)) < 0) {
			continue;
		}

		// Rotate the sensor to OP convention.  The datasheet defines X as towards the right
		// and Y as forward.  OP convention transposes this.  Also the Z is defined negatively
		// to our convention

		// Currently we only support rotations on top so switch X/Y accordingly
		struct sensor_accel_data accel_data;
		struct sensor_gyro_data gyro_data;

		switch (dev->cfg->orientation) {
		case MPU60X0_TOP_0DEG:
			accel_data.y = (int16_t)(mpu9150_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_XOUT_L]);
			accel_data.x = (int16_t)(mpu9150_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_YOUT_L]);
			gyro_data.y  = (int16_t)(mpu9150_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_XOUT_L]);
			gyro_data.x  = (int16_t)(mpu9150_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_YOUT_L]);
			gyro_data.z  = - (int16_t)(mpu9150_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_ZOUT_L]);
			accel_data.z = - (int16_t)(mpu9150_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_ZOUT_L]);
			break;
		case MPU60X0_TOP_90DEG:
			accel_data.y = - (int16_t)(mpu9150_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_YOUT_L]);
			accel_data.x = (int16_t)(mpu9150_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_XOUT_L]);
			gyro_data.y  = - (int16_t)(mpu9150_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_YOUT_L]);
			gyro_data.x  = (int16_t)(mpu9150_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_XOUT_L]);
			gyro_data.z  = - (int16_t)(mpu9150_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_ZOUT_L]);
			accel_data.z = - (int16_t)(mpu9150_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_ZOUT_L]);
			break;
		case MPU60X0_TOP_180DEG:
			accel_data.y = - (int16_t)(mpu9150_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_XOUT_L]);
			accel_data.x = - (int16_t)(mpu9150_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_YOUT_L]);
			gyro_data.y  = - (int16_t)(mpu9150_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_XOUT_L]);
			gyro_data.x  = - (int16_t)(mpu9150_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_YOUT_L]);
			gyro_data.z  = - (int16_t)(mpu9150_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_ZOUT_L]);
			accel_data.z = - (int16_t)(mpu9150_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_ZOUT_L]);
			break;
		case MPU60X0_TOP_270DEG:
			accel_data.y = (int16_t)(mpu9150_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_YOUT_L]);
			accel_data.x = - (int16_t)(mpu9150_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_XOUT_L]);
			gyro_data.y  = (int16_t)(mpu9150_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_YOUT_L]);
			gyro_data.x  = - (int16_t)(mpu9150_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_XOUT_L]);
			gyro_data.z  = - (int16_t)(mpu9150_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_ZOUT_L]);
			accel_data.z = - (int16_t)(mpu9150_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_ZOUT_L]);
			break;
		case MPU60X0_BOTTOM_0DEG:
			accel_data.y = - (int16_t)(mpu9150_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_XOUT_L]);
			accel_data.x = (int16_t)(mpu9150_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_YOUT_L]);
			gyro_data.y  = - (int16_t)(mpu9150_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_XOUT_L]);
			gyro_data.x  = (int16_t)(mpu9150_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_YOUT_L]);
			gyro_data.z  = (int16_t)(mpu9150_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_ZOUT_L]);
			accel_data.z = (int16_t)(mpu9150_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_ZOUT_L]);
			break;
		case MPU60X0_BOTTOM_90DEG:
			accel_data.y = (int16_t)(mpu9150_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_YOUT_L]);
			accel_data.x = (int16_t)(mpu9150_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_XOUT_L]);
			gyro_data.y  = (int16_t)(mpu9150_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_YOUT_L]);
			gyro_data.x  = (int16_t)(mpu9150_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_XOUT_L]);
			gyro_data.z  = (int16_t)(mpu9150_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_ZOUT_L]);
			accel_data.z = (int16_t)(mpu9150_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_ZOUT_L]);
			break;
		case MPU60X0_BOTTOM_180DEG:
			accel_data.y = (int16_t)(mpu9150_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_XOUT_L]);
			accel_data.x = - (int16_t)(mpu9150_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_YOUT_L]);
			gyro_data.y  = (int16_t)(mpu9150_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_XOUT_L]);
			gyro_data.x  = - (int16_t)(mpu9150_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_YOUT_L]);
			gyro_data.z  = (int16_t)(mpu9150_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_ZOUT_L]);
			accel_data.z = (int16_t)(mpu9150_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_ZOUT_L]);
			break;
		case MPU60X0_BOTTOM_270DEG:
			accel_data.y = - (int16_t)(mpu9150_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_YOUT_L]);
			accel_data.x = - (int16_t)(mpu9150_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_XOUT_L]);
			gyro_data.y  = - (int16_t)(mpu9150_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_YOUT_L]);
			gyro_data.x  = - (int16_t)(mpu9150_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_XOUT_L]);
			gyro_data.z  = (int16_t)(mpu9150_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_GYRO_ZOUT_L]);
			accel_data.z = (int16_t)(mpu9150_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu9150_rec_buf[IDX_ACCEL_ZOUT_L]);
			break;
		}


		int16_t raw_temp = (int16_t)(mpu9150_rec_buf[IDX_TEMP_OUT_H] << 8 | mpu9150_rec_buf[IDX_TEMP_OUT_L]);
		float temperature = 35.0f + ((float)raw_temp + 512.0f) / 340.0f;

		// Apply sensor scaling
		float accel_scale = MPU9150_GetAccelScale();
		accel_data.x *= accel_scale;
		accel_data.y *= accel_scale;
		accel_data.z *= accel_scale;
		accel_data.temperature = temperature;

		float gyro_scale = MPU9150_GetGyroScale();
		gyro_data.x *= gyro_scale;
		gyro_data.y *= gyro_scale;
		gyro_data.z *= gyro_scale;
		gyro_data.temperature = temperature;

		OS_QueueSend(dev->accel_queue, &accel_data, 0);
		OS_QueueSend(dev->gyro_queue, &gyro_data, 0);

		// Check for mag data ready.  Reading it clears this flag.
		if ((dev->cfg->use_internal_mag == TRUE) && (MPU9150_Mag_GetReg(MPU9150_MAG_STATUS) > 0)) {
			struct sensor_mag_data mag_data;
			uint8_t mpu9150_mag_buffer[6];
			if (MPU9150_Mag_Read(MPU9150_MAG_XH, mpu9150_mag_buffer, sizeof(mpu9150_mag_buffer)) == 0) {
				switch(dev->cfg->orientation) {
				case MPU60X0_TOP_0DEG:
					mag_data.x = (int16_t) (mpu9150_mag_buffer[1] << 0x08 | mpu9150_mag_buffer[0]);
					mag_data.y = (int16_t) (mpu9150_mag_buffer[3] << 0x08 | mpu9150_mag_buffer[2]);
					mag_data.z = (int16_t) (mpu9150_mag_buffer[5] << 0x08 | mpu9150_mag_buffer[4]);
					break;
				case MPU60X0_TOP_90DEG:
					mag_data.y = (int16_t)  (mpu9150_mag_buffer[1] << 0x08 | mpu9150_mag_buffer[0]);
					mag_data.x = (int16_t) -(mpu9150_mag_buffer[3] << 0x08 | mpu9150_mag_buffer[2]);
					mag_data.z = (int16_t) (mpu9150_mag_buffer[5] << 0x08 | mpu9150_mag_buffer[4]);
					break;
				case MPU60X0_TOP_180DEG:
					mag_data.x = (int16_t) -(mpu9150_mag_buffer[1] << 0x08 | mpu9150_mag_buffer[0]);
					mag_data.y = (int16_t) -(mpu9150_mag_buffer[3] << 0x08 | mpu9150_mag_buffer[2]);
					mag_data.z = (int16_t) (mpu9150_mag_buffer[5] << 0x08 | mpu9150_mag_buffer[4]);
					break;
				case MPU60X0_TOP_270DEG:
					mag_data.y = (int16_t) -(mpu9150_mag_buffer[1] << 0x08 | mpu9150_mag_buffer[0]);
					mag_data.x = (int16_t)  (mpu9150_mag_buffer[3] << 0x08 | mpu9150_mag_buffer[2]);
					mag_data.z = (int16_t) (mpu9150_mag_buffer[5] << 0x08 | mpu9150_mag_buffer[4]);
					break;
				case MPU60X0_BOTTOM_0DEG:
					mag_data.x = (int16_t) (mpu9150_mag_buffer[1] << 0x08 | mpu9150_mag_buffer[0]);
					mag_data.y = (int16_t) - (mpu9150_mag_buffer[3] << 0x08 | mpu9150_mag_buffer[2]);
					mag_data.z = (int16_t) - (mpu9150_mag_buffer[5] << 0x08 | mpu9150_mag_buffer[4]);
					break;
				case MPU60X0_BOTTOM_90DEG:
					mag_data.y = (int16_t) - (mpu9150_mag_buffer[1] << 0x08 | mpu9150_mag_buffer[0]);
					mag_data.x = (int16_t) - (mpu9150_mag_buffer[3] << 0x08 | mpu9150_mag_buffer[2]);
					mag_data.z = (int16_t) - (mpu9150_mag_buffer[5] << 0x08 | mpu9150_mag_buffer[4]);
					break;
				case MPU60X0_BOTTOM_180DEG:
					mag_data.x = (int16_t) - (mpu9150_mag_buffer[1] << 0x08 | mpu9150_mag_buffer[0]);
					mag_data.y = (int16_t)   (mpu9150_mag_buffer[3] << 0x08 | mpu9150_mag_buffer[2]);
					mag_data.z = (int16_t) - (mpu9150_mag_buffer[5] << 0x08 | mpu9150_mag_buffer[4]);
					break;
				case MPU60X0_BOTTOM_270DEG:
					mag_data.y = (int16_t)   (mpu9150_mag_buffer[1] << 0x08 | mpu9150_mag_buffer[0]);
					mag_data.x = (int16_t)   (mpu9150_mag_buffer[3] << 0x08 | mpu9150_mag_buffer[2]);
					mag_data.z = (int16_t) - (mpu9150_mag_buffer[5] << 0x08 | mpu9150_mag_buffer[4]);
					break;
				}

				float mag_scale = MPU9150_GetMagScale();
				mag_data.x *= mag_scale;
				mag_data.y *= mag_scale;
				mag_data.z *= mag_scale;

				// Trigger another measurement
				MPU9150_Mag_SetReg(MPU9150_MAG_CNTR, 0x01);

				OS_QueueSend(dev->mag_queue, &mag_data, 0);
			}
		}

	}
}

#endif

/**
 * @}
 * @}
 */
