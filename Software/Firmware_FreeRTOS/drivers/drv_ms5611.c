#include "drv_ms5611.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"

/* Private constants */
#define MS5611_OVERSAMPLING oversampling
#define MS5611_TASK_PRIORITY	6
#define MS5611_TASK_STACK_BYTES	512

/* MS5611 Addresses */
#define MS5611_I2C_ADDR	        0x77
#define MS5611_RESET            0x1E
#define MS5611_CALIB_ADDR       0xA2  /* First sample is factory stuff */
#define MS5611_CALIB_LEN        16
#define MS5611_ADC_READ         0x00
#define MS5611_PRES_ADDR        0x40
#define MS5611_TEMP_ADDR        0x50
#define MS5611_ADC_MSB          0xF6
#define MS5611_P0               101.3250f

/* Private methods */
static int32_t MS5611_Read(uint8_t address, uint8_t * buffer, uint8_t len);
static int32_t MS5611_WriteCommand(uint8_t command);
static void    MS5611_Task(void *parameters);

/* Private types */

/* Local Types */

enum conversion_type {
	PRESSURE_CONV,
	TEMPERATURE_CONV
};

struct ms5611_dev {
	struct ms5611_cfg * cfg;
	xTaskHandle task;
	xQueueHandle queue;

	int64_t pressure_unscaled;
	int64_t temperature_unscaled;
	uint16_t calibration[6];
	enum conversion_type current_conversion_type;
  bool inited;
	xSemaphoreHandle busy;
};

static struct ms5611_dev *dev;

/**
 * @brief Allocate a new device
 */
static struct ms5611_dev * MS5611_alloc(void)
{
	struct ms5611_dev *ms5611_dev;

	ms5611_dev = (struct ms5611_dev *)pvPortMalloc(sizeof(struct ms5611_dev));
	if (!ms5611_dev)
		return (NULL);

	memset(ms5611_dev, 0, sizeof(struct ms5611_dev));

	ms5611_dev->queue = xQueueCreate(1, sizeof(struct sensor_baro_data));
	if (ms5611_dev->queue == NULL) {
		vPortFree(ms5611_dev);
		return NULL;
	}

	vSemaphporeCreate(ms5611_dev->busy);

	PIOS_Assert(ms5611_dev->busy != NULL);

	return ms5611_dev;
}

/**
 * @brief Validate the handle to the i2c device
 * @returns 0 for valid device or <0 otherwise
 */
static int32_t MS5611_Validate(struct ms5611_dev *dev)
{
	if (dev == NULL)
		return -1;
	if (dev->inited == 0)
		return -2;
	return 0;
}

/**
 * Initialise the MS5611 sensor
 */
int32_t MS5611_Init(struct ms5611_cfg *cfg, int32_t i2c_device)
{
	dev = (struct ms5611_dev *)MS5611_alloc();
	if (dev == NULL)
		return -1;

	dev->cfg = cfg;

	if (MS5611_WriteCommand(MS5611_RESET) != 0)
		return -2;

	vTaskDelay(20);

	uint8_t data[2];

	/* Calibration parameters */
	for (int i = 0; i < NELEMENTS(dev->calibration); i++) {
		MS5611_Read(MS5611_CALIB_ADDR + i * 2, data, 2);
		dev->calibration[i] = (data[0] << 8) | data[1];
	}

	SENSORS_Register(SENSOR_BARO, dev->queue);

	xTaskCreate(MS5611_Task, "pios_ms5611", MS5611_TASK_STACK_BYTES, NULL, MS5611_TASK_PRIORITY,dev->task);
	
	PIOS_Assert(dev->task != NULL);

	return 0;
}

/**
 * Claim the MS5611 device semaphore.
 * \return 0 if no error
 * \return -1 if timeout before claiming semaphore
 */
static int32_t MS5611_ClaimDevice(void)
{
	PIOS_Assert(MS5611_Validate(dev) == 0);

	return xSemaphoreTake(dev->busy, portMAX_DELAY) == pdTRUE ? 0 : 1;
}

/**
 * Release the MS5611 device semaphore.
 * \return 0 if no error
 */
static int32_t MS5611_ReleaseDevice(void)
{
	PIOS_Assert(MS5611_Validate(dev) == 0);

	return xSemaphoreGive(dev->busy) == pdTRUE ? 0 : 1;
}

/**
* Start the ADC conversion
* \param[in] PRESSURE_CONV or TEMPERATURE_CONV to select which measurement to make
* \return 0 for success, -1 for failure (conversion completed and not read)
*/
static int32_t MS5611_StartADC(enum conversion_type type)
{
	if (MS5611_Validate(dev) != 0)
		return -1;

	/* Start the conversion */
	switch (type) {
	case TEMPERATURE_CONV:
		while (MS5611_WriteCommand(MS5611_TEMP_ADDR + dev->cfg->oversampling) != 0)
			continue;
		break;
	case PRESSURE_CONV:
		while (MS5611_WriteCommand(MS5611_PRES_ADDR + dev->cfg->oversampling) != 0)
			continue;
		break;
	default:
		return -1;
	}

	dev->current_conversion_type = type;

	return 0;
}

/**
 * @brief Return the delay for the current osr
 */
static int32_t MS5611_GetDelay()
{
	if (MS5611_Validate(dev) != 0)
		return 100;

	switch(dev->cfg->oversampling) {
	case MS5611_OSR_256:
		return 2;
	case MS5611_OSR_512:
		return 2;
	case MS5611_OSR_1024:
		return 3;
	case MS5611_OSR_2048:
		return 5;
	case MS5611_OSR_4096:
		return 10;
	default:
		break;
	}
	return 10;
}

/**
* Read the ADC conversion value (once ADC conversion has completed)
* \return 0 if successfully read the ADC, -1 if failed
*/
static int32_t MS5611_ReadADC(void)
{
	if (MS5611_Validate(dev) != 0)
		return -1;

	uint8_t data[3];

	static int64_t delta_temp;
	static int64_t temperature;

	/* Read and store the 16bit result */
	if (dev->current_conversion_type == TEMPERATURE_CONV) {
		uint32_t raw_temperature;
		/* Read the temperature conversion */
		if (MS5611_Read(MS5611_ADC_READ, data, 3) != 0)
			return -1;

		raw_temperature = (data[0] << 16) | (data[1] << 8) | data[2];

		delta_temp = (int32_t)raw_temperature - (dev->calibration[4] << 8);
		temperature = 2000 + ((delta_temp * dev->calibration[5]) >> 23);
		dev->temperature_unscaled = temperature;

		// second order temperature compensation
		if (temperature < 2000)
			dev->temperature_unscaled -= (delta_temp * delta_temp) >> 31;

	} else {
		int64_t offset;
		int64_t sens;
		uint32_t raw_pressure;

		/* Read the pressure conversion */
		if (MS5611_Read(MS5611_ADC_READ, data, 3) != 0)
			return -1;

		raw_pressure = (data[0] << 16) | (data[1] << 8) | (data[2] << 0);

		offset = ((int64_t)dev->calibration[1] << 16) + (((int64_t)dev->calibration[3] * delta_temp) >> 7);
		sens = (int64_t)dev->calibration[0] << 15;
		sens = sens + ((((int64_t) dev->calibration[2]) * delta_temp) >> 8);

		// second order temperature compensation
		if (temperature < 2000) {
			offset -= (5 * (temperature - 2000) * (temperature - 2000)) >> 1;
			sens -= (5 * (temperature - 2000) * (temperature - 2000)) >> 2;

			if (dev->temperature_unscaled < -1500) {
				offset -= 7 * (temperature + 1500) * (temperature + 1500);
				sens -= (11 * (temperature + 1500) * (temperature + 1500)) >> 1;
			}
		}

		dev->pressure_unscaled = ((((int64_t)raw_pressure * sens) >> 21) - offset) >> 15;
	}
	return 0;
}

/**
* Reads one or more bytes into a buffer
* \param[in] the command indicating the address to read
* \param[out] buffer destination buffer
* \param[in] len number of bytes which should be read
* \return 0 if operation was successful
* \return -1 if dev is invalid
* \return -2 if error during I2C transfer
*/
static int32_t MS5611_Read(uint8_t address, uint8_t *buffer, uint8_t len)
{
	if (MS5611_Validate(dev) != 0)
		return -1;

	return I2C3_Read(MS5611_I2C_ADDR,address,buffer,len);
}

/**
* Writes one or more bytes to the MS5611
* \param[in] address Register address
* \param[in] buffer source buffer
* \return 0 if operation was successful
* \return -1 if dev is invalid
* \return -2 if error during I2C transfer
*/
static int32_t MS5611_WriteCommand(uint8_t command)
{
	if (MS5611_Validate(dev) != 0)
		return -1;

	return I2C3_Write(MS5611_I2C_ADDR,command,&command,1);
}

/**
* @brief Run self-test operation.
* \return 0 if self-test succeed, -1 if failed
*/
int32_t MS5611_Test()
{
	if (MS5611_Validate(dev) != 0)
		return -1;


	MS5611_ClaimDevice();
	MS5611_StartADC(TEMPERATURE_CONV);
	vTaskDelay(MS5611_GetDelay());
	MS5611_ReadADC();
	MS5611_ReleaseDevice();

	MS5611_ClaimDevice();
	MS5611_StartADC(PRESSURE_CONV);
	vTaskDelay(MS5611_GetDelay());
	MS5611_ReadADC();
	MS5611_ReleaseDevice();

	// check range for sanity according to datasheet
	if (dev->temperature_unscaled < -4000 ||
		dev->temperature_unscaled > 8500 ||
		dev->pressure_unscaled < 1000 ||
		dev->pressure_unscaled > 120000)
		return -1;


	return 0;
}

static void MS5611_Task(void *parameters)
{
	// init this to 1 in order to force a temperature read on the first run
	uint32_t temp_press_interleave_count = 1;
	int32_t  read_adc_result = 0;

	while (1) {

		--temp_press_interleave_count;
		read_adc_result = 0;

		if (temp_press_interleave_count == 0)
		{
			// Update the temperature data
			MS5611_ClaimDevice();
			MS5611_StartADC(TEMPERATURE_CONV);
			vTaskDelay(MS5611_GetDelay());
			read_adc_result = MS5611_ReadADC();
			MS5611_ReleaseDevice();

			temp_press_interleave_count = dev->cfg->temperature_interleaving;
			if (temp_press_interleave_count == 0)
				temp_press_interleave_count = 1;
		}

		// Update the pressure data
		MS5611_ClaimDevice();
		MS5611_StartADC(PRESSURE_CONV);
		vTaskDelay(MS5611_GetDelay());
		read_adc_result = MS5611_ReadADC();
		MS5611_ReleaseDevice();

		// Compute the altitude from the pressure and temperature and send it out
		struct sensor_baro_data data;
		data.temperature = ((float) dev->temperature_unscaled) / 100.0f;
		data.pressure = ((float) dev->pressure_unscaled) / 1000.0f;
		data.altitude = 44330.0f * (1.0f - powf(data.pressure / MS5611_P0, (1.0f / 5.255f)));

		if (read_adc_result == 0) {
			xQueueSend(dev->queue, &data, 0);
		}
	}
}


/**
 * @}
 * @}
 */
