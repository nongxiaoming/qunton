#include "drv_mpu6000.h"
#include "sensors.h"

#define MPU6000_MAX_QUEUESIZE 2

struct mpu6000_dev {
	uint32_t spi_id;
	uint32_t slave_num;
	enum mpu60x0_range gyro_range;
	os_queue_t gyro_queue;
	enum mpu60x0_accel_range accel_range;
	os_queue_t accel_queue;
	struct mpu60x0_cfg *cfg;
	volatile bool_t configured;
	enum mpu60x0_filter filter;
};

//! Global structure for this device device
static struct mpu6000_dev *mpu6000_dev;

//! Private functions
static struct mpu6000_dev *MPU6000_alloc(void);
static int32_t MPU6000_Validate(struct mpu6000_dev *dev);
static void MPU6000_Config(struct mpu60x0_cfg *cfg);
static int32_t MPU6000_ClaimBus(void);
static int32_t MPU6000_ReleaseBus(void);
static int32_t MPU6000_SetReg(uint8_t address, uint8_t buffer);
static int32_t MPU6000_GetReg(uint8_t address);

/**
 * @brief Allocate a new device
 */
static struct mpu6000_dev *MPU6000_alloc(void)
{
	struct mpu6000_dev *mpu6000_dev;

	mpu6000_dev = (struct mpu6000_dev *)OS_Malloc(sizeof(*mpu6000_dev));

	if (!mpu6000_dev) return (NULL);

	mpu6000_dev->configured = FALSE;

	mpu6000_dev->accel_queue = OS_QueueCreate(MPU6000_MAX_QUEUESIZE, sizeof(struct sensor_accel_data));

	if (mpu6000_dev->accel_queue == NULL) {
		OS_Free(mpu6000_dev);
		return NULL;
	}


	mpu6000_dev->gyro_queue = OS_QueueCreate(MPU6000_MAX_QUEUESIZE, sizeof(struct sensor_gyro_data));

	if (mpu6000_dev->gyro_queue == NULL) {
		OS_Free(mpu6000_dev);
		return NULL;
	}

	return mpu6000_dev;
}

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t MPU6000_Validate(struct mpu6000_dev *dev)
{
	if (dev == NULL)
		return -1;

	if (dev->spi_id == 0)
		return -2;

	return 0;
}

/**
 * @brief Initialize the MPU6000 3-axis gyro sensor.
 * @return 0 for success, -1 for failure
 */
int32_t MPU6000_Init(uint32_t spi_id, uint32_t slave_num, struct mpu60x0_cfg *cfg)
{
	mpu6000_dev = MPU6000_alloc();

	if (mpu6000_dev == NULL)
		return -1;

	mpu6000_dev->spi_id = spi_id;
	mpu6000_dev->slave_num = slave_num;
	mpu6000_dev->cfg = cfg;

	/* Configure the MPU6000 Sensor */
	SPI_SetClockSpeed(mpu6000_dev->spi_id, 100000);
  MPU6000_Config(cfg);
	SPI_SetClockSpeed(mpu6000_dev->spi_id, 3000000);

	/* Set up EXTI line */
	EXTI_Init(cfg->exti_cfg);

	SENSORS_Register(SENSOR_ACCEL, mpu6000_dev->accel_queue);

	SENSORS_Register(SENSOR_GYRO, mpu6000_dev->gyro_queue);

	return 0;
}

/**
 * @brief Initialize the MPU6000 3-axis gyro sensor
 * \return none
 * \param[in] MPU6000_ConfigTypeDef struct to be used to configure sensor.
*
*/
static void MPU6000_Config(struct mpu60x0_cfg *cfg)
{
#if defined(MPU6000_SIMPLE_INIT_SEQUENCE)

	// Reset chip registers
	MPU6000_SetReg(MPU60X0_PWR_MGMT_REG, MPU60X0_PWRMGMT_IMU_RST);

	// Reset sensors signal path
	MPU6000_SetReg(MPU60X0_USER_CTRL_REG, MPU60X0_USERCTL_GYRO_RST);

	// Give chip some time to initialize
	OS_Delay(10);

	//Power management configuration
	MPU6000_SetReg(MPU60X0_PWR_MGMT_REG, cfg->Pwr_mgmt_clk);

	// User control
	MPU6000_SetReg(MPU60X0_USER_CTRL_REG, cfg->User_ctl);

	// Digital low-pass filter and scale
	// set this before sample rate else sample rate calculation will fail
	MPU6000_SetLPF(cfg->default_filter);

	// Sample rate
	MPU6000_SetSampleRate(cfg->default_samplerate);

	// Set the gyro scale
	MPU6000_SetGyroRange(MPU60X0_SCALE_500_DEG);

	// Set the accel scale
	MPU6000_SetAccelRange(MPU60X0_ACCEL_8G);

	// Interrupt configuration
	MPU6000_SetReg(MPU60X0_INT_CFG_REG, cfg->interrupt_cfg);

	// Interrupt enable
	MPU6000_SetReg(MPU60X0_INT_EN_REG, cfg->interrupt_en);

#else /* MPU6000_SIMPLE_INIT_SEQUENCE */

	/* This init sequence should really be dropped in favor of something
	 * less redundant but it seems to be hard to get it running well
	 * on all different targets.
	 */

	MPU6000_ClaimBus();
	OS_Delay(1);
	MPU6000_ReleaseBus();
	OS_Delay(10);

	// Reset chip
	MPU6000_SetReg(MPU60X0_PWR_MGMT_REG, 0x80 | cfg->Pwr_mgmt_clk);
	do {
		OS_Delay(5);
	} while (MPU6000_GetReg(MPU60X0_PWR_MGMT_REG) & 0x80);

	OS_Delay(25);

	// Reset chip and fifo
	MPU6000_SetReg(MPU60X0_USER_CTRL_REG, 0x80 | 0x01 | 0x02 | 0x04);
	do {
		OS_Delay(5);
	} while (MPU6000_GetReg(MPU60X0_USER_CTRL_REG) & 0x07);

	OS_Delay(25);

	//Power management configuration
	MPU6000_SetReg(MPU60X0_PWR_MGMT_REG, cfg->Pwr_mgmt_clk);

	// Interrupt configuration
	MPU6000_SetReg(MPU60X0_INT_CFG_REG, cfg->interrupt_cfg);

	// Interrupt configuration
	MPU6000_SetReg(MPU60X0_INT_EN_REG, cfg->interrupt_en);

	// Set the accel scale
	MPU6000_SetAccelRange(MPU60X0_ACCEL_8G);

	// Digital low-pass filter and scale
	// set this before sample rate else sample rate calculation will fail
	MPU6000_SetLPF(cfg->default_filter);

	// Sample rate
	MPU6000_SetSampleRate(cfg->default_samplerate);

	// Set the gyro scale
	MPU6000_SetGyroRange(MPU60X0_SCALE_500_DEG);

	// Interrupt configuration
	MPU6000_SetReg(MPU60X0_USER_CTRL_REG, cfg->User_ctl);

	//Power management configuration
	MPU6000_SetReg(MPU60X0_PWR_MGMT_REG, cfg->Pwr_mgmt_clk);

	// Interrupt configuration
	MPU6000_SetReg(MPU60X0_INT_CFG_REG, cfg->interrupt_cfg);

	// Interrupt configuration
	MPU6000_SetReg(MPU60X0_INT_EN_REG, cfg->interrupt_en);

#endif /* MPU6000_SIMPLE_INIT_SEQUENCE */

	mpu6000_dev->configured = TRUE;
}

/**
 * Set the gyro range and store it locally for scaling
 */
void MPU6000_SetGyroRange(enum mpu60x0_range gyro_range)
{
	MPU6000_SetReg(MPU60X0_GYRO_CFG_REG, gyro_range);

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

	mpu6000_dev->gyro_range = gyro_range;
}

/**
 * Set the accel range and store it locally for scaling
 */

void PIOS_MPU6000_SetAccelRange(enum mpu60x0_accel_range accel_range)
{
	MPU6000_SetReg(MPU60X0_ACCEL_CFG_REG, accel_range);

	mpu6000_dev->accel_range = accel_range;
}

/**
 * Set the sample rate in Hz by determining the nearest divisor
 * @param[in] sample rate in Hz
 */
void MPU6000_SetSampleRate(uint16_t samplerate_hz)
{
	uint16_t filter_frequency = 8000;

	if (mpu6000_dev->filter != MPU60X0_LOWPASS_256_HZ)
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

	MPU6000_SetReg(MPU60X0_SMPLRT_DIV_REG, (uint8_t)divisor);
}

/**
 * Configure the digital low-pass filter
 */
void MPU6000_SetLPF(enum mpu60x0_filter filter)
{
	MPU6000_SetReg(MPU60X0_DLPF_CFG_REG, filter);

	mpu6000_dev->filter = filter;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t MPU6000_ClaimBus()
{
	if (MPU6000_Validate(mpu6000_dev) != 0)
		return -1;

	if (SPI_ClaimBus(mpu6000_dev->spi_id) != 0)
		return -2;

	SPI_RC_PinSet(mpu6000_dev->spi_id, mpu6000_dev->slave_num, 0);
	return 0;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * \param[in] pointer which receives if a task has been woken
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t MPU6000_ClaimBusISR(bool_t *woken)
{
	if (MPU6000_Validate(mpu6000_dev) != 0)
		return -1;

	if (SPI_ClaimBusISR(mpu6000_dev->spi_id, woken) != 0)
		return -2;

	SPI_RC_PinSet(mpu6000_dev->spi_id, mpu6000_dev->slave_num, 0);
	return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful
 */
static int32_t MPU6000_ReleaseBus()
{
	if (MPU6000_Validate(mpu6000_dev) != 0)
		return -1;

	SPI_RC_PinSet(mpu6000_dev->spi_id, mpu6000_dev->slave_num, 1);

	return SPI_ReleaseBus(mpu6000_dev->spi_id);
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * \param[in] pointer which receives if a task has been woken
 * @return 0 if successful
 */
static int32_t MPU6000_ReleaseBusISR(bool_t *woken)
{
	if (MPU6000_Validate(mpu6000_dev) != 0)
		return -1;

	SPI_RC_PinSet(mpu6000_dev->spi_id, mpu6000_dev->slave_num, 1);

	return SPI_ReleaseBusISR(mpu6000_dev->spi_id, woken);
}

/**
 * @brief Read a register from MPU6000
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t MPU6000_GetReg(uint8_t reg)
{
	uint8_t data;

	if (MPU6000_ClaimBus() != 0)
		return -1;

	SPI_TransferByte(mpu6000_dev->spi_id, (0x80 | reg)); // request byte
	data = SPI_TransferByte(mpu6000_dev->spi_id, 0);     // receive response

	MPU6000_ReleaseBus();
	return data;
}

/**
 * @brief Writes one byte to the MPU6000
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int32_t MPU6000_SetReg(uint8_t reg, uint8_t data)
{
	if (MPU6000_ClaimBus() != 0)
		return -1;

	if (SPI_TransferByte(mpu6000_dev->spi_id, 0x7f & reg) != 0) {
		MPU6000_ReleaseBus();
		return -2;
	}

	if (SPI_TransferByte(mpu6000_dev->spi_id, data) != 0) {
		MPU6000_ReleaseBus();
		return -3;
	}

	MPU6000_ReleaseBus();

	return 0;
}

/*
 * @brief Read the identification bytes from the MPU6000 sensor
 * \return ID read from MPU6000 or -1 if failure
*/
static int32_t MPU6000_ReadID()
{
	int32_t mpu6000_id = MPU6000_GetReg(MPU60X0_WHOAMI);

	if (mpu6000_id < 0)
		return -1;

	return mpu6000_id;
}

/**
 * Get the gyro scale based on the active device settings
 * @return Scale in (deg/s) / LSB
 */
static float MPU6000_GetGyroScale()
{
	switch (mpu6000_dev->gyro_range) {
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

/**
 * Get the accel scale based on the active settings
 * @returns Scale in (m/s^2) / LSB
 */
static float MPU6000_GetAccelScale()
{
	switch (mpu6000_dev->accel_range) {
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


/**
 * @brief Run self-test operation.
 * \return 0 if test succeeded
 * \return non-zero value if test succeeded
 */
int32_t MPU6000_Test(void)
{
	/* Verify that ID matches (MPU6000 ID is 0x68) */
	int32_t mpu6000_id = MPU6000_ReadID();

	if (mpu6000_id < 0)
		return -1;

	if (mpu6000_id != 0x68)
		return -2;

	return 0;
}

/**
* @brief IRQ Handler.  Read all the data from onboard buffer
*/
bool_t MPU6000_IRQHandler(void)
{
	if (MPU6000_Validate(mpu6000_dev) != 0 || mpu6000_dev->configured == FALSE)
		return FALSE;

	bool_t woken = FALSE;

	if (MPU6000_ClaimBusISR(&woken) != 0)
		return FALSE;

	enum {
	    IDX_SPI_DUMMY_BYTE = 0,
	    IDX_ACCEL_XOUT_H,
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

	uint8_t mpu6000_send_buf[BUFFER_SIZE] = { MPU60X0_ACCEL_X_OUT_MSB | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t mpu6000_rec_buf[BUFFER_SIZE];

	if (SPI_TransferBlock(mpu6000_dev->spi_id, mpu6000_send_buf, mpu6000_rec_buf, sizeof(mpu6000_send_buf), NULL) < 0) {
		MPU6000_ReleaseBusISR(&woken);
		return FALSE;
	}

	MPU6000_ReleaseBusISR(&woken);


	// Rotate the sensor to OP convention.  The datasheet defines X as towards the right
	// and Y as forward.  OP convention transposes this.  Also the Z is defined negatively
	// to our convention

	// Currently we only support rotations on top so switch X/Y accordingly
	struct sensor_accel_data accel_data;
	struct sensor_gyro_data gyro_data;

	switch (mpu6000_dev->cfg->orientation) {
	case MPU60X0_TOP_0DEG:
		accel_data.y = (int16_t)(mpu6000_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_XOUT_L]);
		accel_data.x = (int16_t)(mpu6000_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_YOUT_L]);
		gyro_data.y  = (int16_t)(mpu6000_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_XOUT_L]);
		gyro_data.x  = (int16_t)(mpu6000_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_YOUT_L]);
		gyro_data.z  = - (int16_t)(mpu6000_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_ZOUT_L]);
		accel_data.z = - (int16_t)(mpu6000_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_ZOUT_L]);
		break;
	case MPU60X0_TOP_90DEG:
		accel_data.y = - (int16_t)(mpu6000_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_YOUT_L]);
		accel_data.x = (int16_t)(mpu6000_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_XOUT_L]);
		gyro_data.y  = - (int16_t)(mpu6000_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_YOUT_L]);
		gyro_data.x  = (int16_t)(mpu6000_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_XOUT_L]);
		gyro_data.z  = - (int16_t)(mpu6000_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_ZOUT_L]);
		accel_data.z = - (int16_t)(mpu6000_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_ZOUT_L]);
		break;
	case MPU60X0_TOP_180DEG:
		accel_data.y = - (int16_t)(mpu6000_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_XOUT_L]);
		accel_data.x = - (int16_t)(mpu6000_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_YOUT_L]);
		gyro_data.y  = - (int16_t)(mpu6000_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_XOUT_L]);
		gyro_data.x  = - (int16_t)(mpu6000_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_YOUT_L]);
		gyro_data.z  = - (int16_t)(mpu6000_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_ZOUT_L]);
		accel_data.z = - (int16_t)(mpu6000_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_ZOUT_L]);
		break;
	case MPU60X0_TOP_270DEG:
		accel_data.y = (int16_t)(mpu6000_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_YOUT_L]);
		accel_data.x = - (int16_t)(mpu6000_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_XOUT_L]);
		gyro_data.y  = (int16_t)(mpu6000_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_YOUT_L]);
		gyro_data.x  = - (int16_t)(mpu6000_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_XOUT_L]);
		gyro_data.z  = - (int16_t)(mpu6000_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_ZOUT_L]);
		accel_data.z = - (int16_t)(mpu6000_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_ZOUT_L]);
		break;
	case MPU60X0_BOTTOM_0DEG:
		accel_data.y = - (int16_t)(mpu6000_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_XOUT_L]);
		accel_data.x = (int16_t)(mpu6000_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_YOUT_L]);
		gyro_data.y  = - (int16_t)(mpu6000_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_XOUT_L]);
		gyro_data.x  = (int16_t)(mpu6000_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_YOUT_L]);
		gyro_data.z  = (int16_t)(mpu6000_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_ZOUT_L]);
		accel_data.z = (int16_t)(mpu6000_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_ZOUT_L]);
		break;
	case MPU60X0_BOTTOM_90DEG:
		accel_data.y = (int16_t)(mpu6000_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_YOUT_L]);
		accel_data.x = - (int16_t)(mpu6000_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_XOUT_L]);
		gyro_data.y  = (int16_t)(mpu6000_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_YOUT_L]);
		gyro_data.x  = - (int16_t)(mpu6000_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_XOUT_L]);
		gyro_data.z  = (int16_t)(mpu6000_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_ZOUT_L]);
		accel_data.z = (int16_t)(mpu6000_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_ZOUT_L]);
		break;	
	case MPU60X0_BOTTOM_180DEG:
		accel_data.y = (int16_t)(mpu6000_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_XOUT_L]);
		accel_data.x = - (int16_t)(mpu6000_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_YOUT_L]);
		gyro_data.y  = (int16_t)(mpu6000_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_XOUT_L]);
		gyro_data.x  = - (int16_t)(mpu6000_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_YOUT_L]);
		gyro_data.z  = (int16_t)(mpu6000_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_ZOUT_L]);
		accel_data.z = (int16_t)(mpu6000_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_ZOUT_L]);
		break;
	case MPU60X0_BOTTOM_270DEG:
		accel_data.y = - (int16_t)(mpu6000_rec_buf[IDX_ACCEL_YOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_YOUT_L]);
		accel_data.x = (int16_t)(mpu6000_rec_buf[IDX_ACCEL_XOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_XOUT_L]);
		gyro_data.y  = - (int16_t)(mpu6000_rec_buf[IDX_GYRO_YOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_YOUT_L]);
		gyro_data.x  = (int16_t)(mpu6000_rec_buf[IDX_GYRO_XOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_XOUT_L]);
		gyro_data.z  = (int16_t)(mpu6000_rec_buf[IDX_GYRO_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_GYRO_ZOUT_L]);
		accel_data.z = (int16_t)(mpu6000_rec_buf[IDX_ACCEL_ZOUT_H] << 8 | mpu6000_rec_buf[IDX_ACCEL_ZOUT_L]);	
		break;
	}


	int16_t raw_temp = (int16_t)(mpu6000_rec_buf[IDX_TEMP_OUT_H] << 8 | mpu6000_rec_buf[IDX_TEMP_OUT_L]);
	float temperature = 35.0f + ((float)raw_temp + 512.0f) / 340.0f;

	// Apply sensor scaling
	float accel_scale = MPU6000_GetAccelScale();
	accel_data.x *= accel_scale;
	accel_data.y *= accel_scale;
	accel_data.z *= accel_scale;
	accel_data.temperature = temperature;

	float gyro_scale = MPU6000_GetGyroScale();
	gyro_data.x *= gyro_scale;
	gyro_data.y *= gyro_scale;
	gyro_data.z *= gyro_scale;
	gyro_data.temperature = temperature;

	OS_QueueSendFromISR(mpu6000_dev->accel_queue, &accel_data, &woken);

  OS_QueueSendFromISR(mpu6000_dev->gyro_queue, &gyro_data, &woken);

	return woken;

}


/**
 * @}
 * @}
 */
