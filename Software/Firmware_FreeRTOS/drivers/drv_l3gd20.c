#include "stm32f4xx.h"
#include "drv_l3gd20.h"
#include "sensors.h"

#define L3GD20_QUEUESIZE 2

//! Local types
struct l3gd20_dev {
	uint32_t spi_id;
	uint32_t slave_num;
	os_queue_t queue;
	struct l3gd20_cfg *cfg;
	enum l3gd20_filter bandwidth;
	enum l3gd20_range range;
	volatile bool_t configured;
};

struct l3gd20_data {
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int16_t temperature;
};

//! Global structure for this device device
static struct l3gd20_dev *l3gd20_dev;

//! Private functions
static struct l3gd20_dev *L3GD20_alloc(void);
static int32_t L3GD20_Validate(struct l3gd20_dev *dev);
static int32_t L3GD20_Config(struct l3gd20_cfg *cfg);
static int32_t L3GD20_SetReg(uint8_t address, uint8_t buffer);
static int32_t L3GD20_GetReg(uint8_t address);
static int32_t L3GD20_GetRegIsr(uint8_t address, bool_t *woken);
static int32_t L3GD20_ClaimBus(void);
static int32_t L3GD20_ClaimBusIsr(bool_t *woken);
static int32_t L3GD20_ReleaseBus(void);
static int32_t L3GD20_ReleaseBusIsr(bool_t *woken);
static int32_t L3GD20_ReadGyros(struct l3gd20_data *buffer);

/* Local Variables */

/**
 * @brief Allocate a new device
 */
static struct l3gd20_dev *L3GD20_alloc(void)
{
	struct l3gd20_dev *l3gd20_dev;

	l3gd20_dev = (struct l3gd20_dev *)OS_Malloc(sizeof(*l3gd20_dev));

	if (!l3gd20_dev) return (NULL);

	l3gd20_dev->configured = FALSE;

	l3gd20_dev->queue = OS_QueueCreate(L3GD20_QUEUESIZE, sizeof(struct sensor_gyro_data));

	if (l3gd20_dev->queue == NULL) {
		OS_Free(l3gd20_dev);
		return NULL;
	}

	return l3gd20_dev;
}

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t L3GD20_Validate(struct l3gd20_dev *dev)
{
	if (dev == NULL)
		return -1;

	if (dev->spi_id == 0)
		return -2;

	return 0;
}

/**
 * @brief Initialize the L3GD20 3-axis gyro sensor.
 * @return 0 for success, -1 for failure to allocate, -2 for failure to get irq
 */
int32_t L3GD20_Init(uint32_t spi_id, uint32_t slave_num, struct l3gd20_cfg *cfg)
{
	l3gd20_dev = L3GD20_alloc();

	if (l3gd20_dev == NULL)
		return -1;

	l3gd20_dev->spi_id = spi_id;
	l3gd20_dev->slave_num = slave_num;
	l3gd20_dev->cfg = cfg;

	/* Configure the L3GD20 Sensor */
	if(L3GD20_Config(cfg) != 0)
		return -2;

	/* Set up EXTI */
	EXTI_Init(cfg->exti_cfg);

	// An initial read is needed to get it running
	struct l3gd20_data data;
	L3GD20_ReadGyros(&data);

	SENSORS_Register(SENSOR_GYRO, l3gd20_dev->queue);

	return 0;
}

/**
 * @brief Initialize the L3GD20 3-axis gyro sensor
 * \return 0 for successful configuration or -1 otherwise
 * \param[in] L3GD20_ConfigTypeDef struct to be used to configure sensor.
*
*/
static int32_t L3GD20_Config(struct l3gd20_cfg *cfg)
{
	// This register enables the channels
	while (L3GD20_SetReg(L3GD20_CTRL_REG1, L3GD20_RATE_380HZ_100HZ |
	                          L3GD20_CTRL1_PD | L3GD20_CTRL1_ZEN |
	                          L3GD20_CTRL1_YEN | L3GD20_CTRL1_XEN) != 0);

	// Disable the high pass filters
	while (L3GD20_SetReg(L3GD20_CTRL_REG2, 0) != 0);

	// Set int2 to go high on data ready
	while (L3GD20_SetReg(L3GD20_CTRL_REG3, 0x08) != 0);

	// Select SPI interface, 500 deg/s, endianness?
	while (L3GD20_SetRange(cfg->range) != 0);

	// disable HPF
	while (L3GD20_SetReg(L3GD20_CTRL_REG5, 0x00) != 0);

	l3gd20_dev->configured = TRUE;

	return 0;
}

/**
 * @brief Sets the maximum range of the L3GD20
 * @returns 0 for success, -1 for invalid device, -2 if unable to set register
 */
int32_t L3GD20_SetRange(enum l3gd20_range range)
{
	if (L3GD20_Validate(l3gd20_dev) != 0)
		return -1;

	l3gd20_dev->range = range;

	if (L3GD20_SetReg(L3GD20_CTRL_REG4, l3gd20_dev->range) != 0)
		return -2;

	switch(range) {
	case L3GD20_SCALE_250_DEG:
		SENSORS_SetMaxGyro(250);
		break;
	case L3GD20_SCALE_500_DEG:
		SENSORS_SetMaxGyro(500);
		break;
	case L3GD20_SCALE_2000_DEG:
		SENSORS_SetMaxGyro(2000);
		break;
	}

	return 0;
}

/**
 * @brief Set the sample rate, 760 or 380
 * @param[in] enum l3gd20_rate
 * @return 0 if successful, -1 for invalid device, -2 if unable to get register, -3 if unable to set register
 */
int32_t L3GD20_SetSampleRate(enum l3gd20_rate rate)
{
	if (L3GD20_Validate(l3gd20_dev) != 0)
		return -1;

	int32_t l3gd20_reg1 = L3GD20_GetReg(L3GD20_CTRL_REG1) & 0x0F;

	if (l3gd20_reg1 == -1)
		return -2;

	if (L3GD20_SetReg(L3GD20_CTRL_REG1, rate | l3gd20_reg1) != 0)
		return -3;

	return 0;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t L3GD20_ClaimBus()
{
	if (L3GD20_Validate(l3gd20_dev) != 0)
		return -1;

	if (SPI_ClaimBus(l3gd20_dev->spi_id) != 0)
		return -2;

	SPI_RC_PinSet(l3gd20_dev->spi_id, l3gd20_dev->slave_num, 0);

	return 0;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * \param[in] pointer which receives if a task has been woken
 * @return 0 if successful, -1 for invalid device, -2 if unable to claim bus
 */
static int32_t L3GD20_ClaimBusIsr(bool_t *woken)
{
	if (L3GD20_Validate(l3gd20_dev) != 0)
		return -1;

	if (SPI_ClaimBusISR(l3gd20_dev->spi_id, woken) < 0)
		return -2;

	SPI_RC_PinSet(l3gd20_dev->spi_id, l3gd20_dev->slave_num, 0);

	return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful, -1 for invalid device
 */
static int32_t L3GD20_ReleaseBus(void)
{
	if (L3GD20_Validate(l3gd20_dev) != 0)
		return -1;

	SPI_RC_PinSet(l3gd20_dev->spi_id, l3gd20_dev->slave_num, 1);

	return SPI_ReleaseBus(l3gd20_dev->spi_id);
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * \param[in] pointer which receives if a task has been woken
 * @return 0 if successful, -1 for invalid device
 */
static int32_t L3GD20_ReleaseBusIsr(bool_t *woken)
{
	if (L3GD20_Validate(l3gd20_dev) != 0)
		return -1;

	SPI_RC_PinSet(l3gd20_dev->spi_id, l3gd20_dev->slave_num, 1);

	return SPI_ReleaseBusISR(l3gd20_dev->spi_id, woken);
}

/**
 * @brief Read a register from L3GD20
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
static int32_t L3GD20_GetReg(uint8_t reg)
{
	uint8_t data;

	if (L3GD20_ClaimBus() != 0)
		return -1;

	SPI_TransferByte(l3gd20_dev->spi_id, (0x80 | reg)); // request byte
	data = SPI_TransferByte(l3gd20_dev->spi_id, 0);     // receive response

	L3GD20_ReleaseBus();

	return data;
}

/**
 * @brief Read a register from L3GD20 from ISR context
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 * \param[in] task woken
 */
static int32_t L3GD20_GetRegIsr(uint8_t reg, bool_t *woken)
{
	uint8_t data;

	if (L3GD20_ClaimBusIsr(woken) != 0)
		return -1;

	SPI_TransferByte(l3gd20_dev->spi_id, (0x80 | reg)); // request byte
	data = SPI_TransferByte(l3gd20_dev->spi_id, 0);     // receive response

	L3GD20_ReleaseBusIsr(woken);

	return data;
}

/**
 * @brief Writes one byte to the L3GD20
 * \param[in] reg Register address
 * \param[in] data Byte to write
 * \return 0 if operation was successful
 * \return -1 if unable to claim SPI bus
 * \return -2 if unable to claim i2c device
 */
static int32_t L3GD20_SetReg(uint8_t reg, uint8_t data)
{
	if (L3GD20_ClaimBus() != 0)
		return -1;

	SPI_TransferByte(l3gd20_dev->spi_id, 0x7f & reg);
	SPI_TransferByte(l3gd20_dev->spi_id, data);

	L3GD20_ReleaseBus();

	return 0;
}

/**
 * @brief Read current X, Z, Y values (in that order)
 * \param[out] int16_t array of size 3 to store X, Z, and Y magnetometer readings
 * \returns The number of samples remaining in the fifo
 */
static int32_t L3GD20_ReadGyros(struct l3gd20_data *data)
{
	uint8_t buf[7] = { L3GD20_GYRO_X_OUT_LSB | 0x80 | 0x40, 0, 0, 0, 0, 0, 0 };
	uint8_t rec[7];

	if (L3GD20_ClaimBus() != 0)
		return -1;

	if (SPI_TransferBlock(l3gd20_dev->spi_id, &buf[0], &rec[0], sizeof(buf), NULL) < 0) {
		L3GD20_ReleaseBus();
		data->gyro_x = 0;
		data->gyro_y = 0;
		data->gyro_z = 0;
		data->temperature = 0;
		return -2;
	}

	L3GD20_ReleaseBus();

	memcpy((uint8_t *)&data->gyro_x, &rec[1], 6);
	data->temperature = L3GD20_GetReg(L3GD20_OUT_TEMP);

	return 0;
}

/**
 * @brief Read the identification bytes from the L3GD20 sensor
 * \return ID read from L3GD20 or -1 if failure
*/
int32_t L3GD20_ReadID()
{
	int32_t l3gd20_id = L3GD20_GetReg(L3GD20_WHOAMI);

	if (l3gd20_id < 0)
		return -1;

	return l3gd20_id;
}

static float L3GD20_GetScale()
{
	if (L3GD20_Validate(l3gd20_dev) != 0)
		return -1;

	switch (l3gd20_dev->range) {
	case L3GD20_SCALE_250_DEG:
		return 0.00875f;
	case L3GD20_SCALE_500_DEG:
		return 0.01750f;
	case L3GD20_SCALE_2000_DEG:
		return 0.070f;
	}

	return 0;
}

/**
 * @brief Run self-test operation.
 * \return 0 if test succeeded
 * \return non-zero value if test succeeded
 */
uint8_t L3GD20_Test(void)
{
	int32_t l3gd20_id = L3GD20_ReadID();

	if (l3gd20_id < 0)
		return -1;

	uint8_t id = l3gd20_id;

	if (id == 0xD4)
		return 0;

	return -2;
}

/**
* @brief IRQ Handler.  Read all the data from onboard buffer
*/
bool_t L3GD20_IRQHandler(void)
{
	if (L3GD20_Validate(l3gd20_dev) != 0 || l3gd20_dev->configured == FALSE)
		return FALSE;

	struct l3gd20_data data;
	uint8_t buf[7] = { L3GD20_GYRO_X_OUT_LSB | 0x80 | 0x40, 0, 0, 0, 0, 0, 0 };
	uint8_t rec[7];

	/* This code duplicates ReadGyros above but uses ClaimBusIsr */
	bool_t woken = FALSE;

	if (L3GD20_ClaimBusIsr(&woken) != 0)
		return woken;

	if (SPI_TransferBlock(l3gd20_dev->spi_id, &buf[0], &rec[0], sizeof(buf), NULL) < 0) {
		L3GD20_ReleaseBusIsr(&woken);
		return woken;
	}

	L3GD20_ReleaseBusIsr(&woken);

	memcpy((uint8_t *)&data.gyro_x, &rec[1], 6);


	struct sensor_gyro_data normalized_data;
	float scale = L3GD20_GetScale();
	normalized_data.y = data.gyro_x * scale;
	normalized_data.x = data.gyro_y * scale;
	normalized_data.z = data.gyro_z * scale;
	normalized_data.temperature = L3GD20_GetRegIsr(L3GD20_OUT_TEMP, &woken);

	Queue_Send_FromISR(l3gd20_dev->queue, &normalized_data, &woken);

	return woken;
}


/**
 * @}
 * @}
 */
