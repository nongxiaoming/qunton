#include "drv_bma180.h"

#define BMA180_MAX_DOWNSAMPLE 10
struct bma180_dev {
	uint32_t spi_id;
	uint32_t slave_num;
	int16_t buffer[BMA180_MAX_DOWNSAMPLE * sizeof(struct bma180_data)];
	t_fifo_buffer fifo;
	const struct bma180_cfg * cfg;
	enum bma180_bandwidth bandwidth;
	enum bma180_range range;
};

//! Global structure for this device device
static struct bma180_dev * dev;

//! Private functions
static struct bma180_dev * BMA180_alloc(void);
static int32_t BMA180_Validate(struct bma180_dev * dev);
static int32_t BMA180_GetReg(uint8_t reg);
static int32_t BMA180_SetReg(uint8_t reg, uint8_t data);
static int32_t BMA180_SelectBW(enum bma180_bandwidth bw);
static int32_t BMA180_SetRange(enum bma180_range range);
static int32_t BMA180_Config();
static int32_t BMA180_EnableIrq();

/**
 * @brief Allocate a new device
 */
static struct bma180_dev * BMA180_alloc(void)
{
	struct bma180_dev * bma180_dev;
	
	bma180_dev = (struct bma180_dev *)OS_Malloc(sizeof(*bma180_dev));
	if (!bma180_dev) return (NULL);
	
	fifoBuf_init(&bma180_dev->fifo, (uint8_t *) bma180_dev->buffer, sizeof(bma180_dev->buffer));
	
	return(bma180_dev);
}

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t BMA180_Validate(struct bma180_dev * dev)
{
	if (dev == NULL) 
		return -1;
	if (dev->spi_id == 0)
		return -2;
	return 0;
}

/**
 * @brief Initialize with good default settings
 * @returns 0 for success, -1 for failure
 */
int32_t BMA180_Init(uint32_t spi_id, uint32_t slave_num, const struct bma180_cfg * cfg)
{	
	dev = BMA180_alloc();
	if(dev == NULL)
		return -1;
		
	dev->spi_id = spi_id;
	dev->slave_num = slave_num;
	dev->cfg = cfg;
	
	if(BMA180_Config() < 0)
		return -1;
	OS_Delay(1);
	
	EXTI_Init(dev->cfg->exti_cfg);

	while(BMA180_EnableIrq() != 0);
	
	return 0;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 if unable to claim bus
 */
int32_t BMA180_ClaimBus(void)
{
	if(BMA180_Validate(dev) != 0)
		return -1;

	if(SPI_ClaimBus(dev->spi_id) != 0) {
		return -1;
	}

	SPI_RC_PinSet(dev->spi_id,dev->slave_num,0);

	return 0;
}

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * \param[in] pointer which receives if a task has been woken
 * @return 0 if successful, -1 if unable to claim bus
 */
int32_t BMA180_ClaimBusISR(bool *woken)
{
	if(BMA180_Validate(dev) != 0)
		return -1;

	if(SPI_ClaimBusISR(dev->spi_id, woken) != 0) {
		return -1;
	}

	SPI_RC_PinSet(dev->spi_id,dev->slave_num,0);
	return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful
 */
int32_t BMA180_ReleaseBus()
{
	if(BMA180_Validate(dev) != 0)
		return -1;

	SPI_RC_PinSet(dev->spi_id,dev->slave_num,1);

	return SPI_ReleaseBus(dev->spi_id);
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * \param[in] pointer which receives if a task has been woken
 * @return 0 if successful
 */
int32_t BMA180_ReleaseBusISR(bool *woken)
{
	if(BMA180_Validate(dev) != 0)
		return -1;

	SPI_RC_PinSet(dev->spi_id,dev->slave_num,1);

	return SPI_ReleaseBusISR(dev->spi_id, woken);
}

/**
 * @brief Read a register from BMA180
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
int32_t BMA180_GetReg(uint8_t reg)
{
	if(BMA180_Validate(dev) != 0)
		return -1;

	uint8_t data;
	
	if(BMA180_ClaimBus() != 0)
		return -1;	

	SPI_TransferByte(dev->spi_id,(0x80 | reg) ); // request byte
	data = SPI_TransferByte(dev->spi_id,0 );     // receive response
	
	BMA180_ReleaseBus();
	return data;
}

/**
 * @brief Write a BMA180 register.  EEPROM must be unlocked before calling this function.
 * @return none
 * @param reg[in] address of register to be written
 * @param data[in] data that is to be written to register
 */
int32_t BMA180_SetReg(uint8_t reg, uint8_t data)
{
	if(BMA180_ClaimBus() != 0)
		return -1;
	
	SPI_TransferByte(dev->spi_id, 0x7f & reg);
	SPI_TransferByte(dev->spi_id, data);

	BMA180_ReleaseBus();
	
	return 0;
}


static int32_t BMA180_EnableEeprom() {
	// Enable EEPROM writing
	int32_t byte = BMA180_GetReg(BMA_CTRREG0);
	if(byte < 0)
		return -1;
	byte |= 0x10;                                      // Set bit 4
	if(BMA180_SetReg(BMA_CTRREG0,(uint8_t) byte) < 0)    // Have to set ee_w to		
		return -1;
	return 0;
}

static int32_t BMA180_DisableEeprom() {
	// Enable EEPROM writing
	int32_t byte = BMA180_GetReg(BMA_CTRREG0);
	if(byte < 0)
		return -1;
	byte |= 0x10;                                      // Set bit 4
	if(BMA180_SetReg(BMA_CTRREG0,(uint8_t) byte) < 0)    // Have to set ee_w to		
		return -1;
	return 0;
}

/**
 * @brief Set the default register settings
 * @return 0 if successful, -1 if not
 */
static int32_t BMA180_Config(void) 
{
	/*
	0x35 = 0x81  //smp-skip = 1 for less interrupts
	0x33 = 0x81  //shadow-dis = 1, update MSB and LSB synchronously
	0x27 = 0x01  //dis-i2c
	0x21 = 0x02  //new_data_int = 1
	 */
		
	OS_Delay(1);

	if(BMA180_Validate(dev) != 0) 
		return -1;

	while(BMA180_EnableEeprom() != 0);
	while(BMA180_SetReg(BMA_RESET, BMA_RESET_CODE) != 0);
	while(BMA180_SetReg(BMA_OFFSET_LSB1, 0x81) != 0);
	while(BMA180_SetReg(BMA_GAIN_Y, 0x81) != 0);
	while(BMA180_SetReg(BMA_CTRREG3, 0xFF) != 0);
	while(BMA180_SelectBW(dev->cfg->bandwidth) != 0);
	while(BMA180_SetRange(dev->cfg->range) != 0);
	while(BMA180_DisableEeprom() != 0);

	return 0;
}

/**
 * @brief Select the bandwidth the digital filter pass allows.
 * @return 0 if successful, -1 if not
 * @param rate[in] Bandwidth setting to be used
 * 
 * EEPROM must be write-enabled before calling this function.
 */
static int32_t BMA180_SelectBW(enum bma180_bandwidth bw)
{
	if(BMA180_Validate(dev) != 0)
		return -1;

	dev->bandwidth = bw;
	
	uint8_t reg;
	reg = BMA180_GetReg(BMA_BW_ADDR);
	reg = (reg & ~BMA_BW_MASK) | ((bw << BMA_BW_SHIFT) & BMA_BW_MASK);
	return BMA180_SetReg(BMA_BW_ADDR, reg);
}

/**
 * @brief Select the full scale acceleration range.
 * @return 0 if successful, -1 if not
 * @param rate[in] Range setting to be used
 *
 */
static int32_t BMA180_SetRange(enum bma180_range new_range) 
{	
	if(BMA180_Validate(dev) != 0)
		return -1;

	uint8_t reg;
		
	dev->range = new_range;
	reg = BMA180_GetReg(BMA_RANGE_ADDR);
	reg = (reg & ~BMA_RANGE_MASK) | ((dev->range << BMA_RANGE_SHIFT) & BMA_RANGE_MASK);
	return BMA180_SetReg(BMA_RANGE_ADDR, reg);
}

static int32_t BMA180_EnableIrq() 
{

	if(BMA180_EnableEeprom() < 0)
		return -1;

	if(BMA180_SetReg(BMA_CTRREG3, BMA_NEW_DAT_INT) < 0)
		return -1;

	if(BMA180_DisableEeprom() < 0)
		return -1;

	return 0;
}

/**
 * @brief Read a single set of values from the x y z channels
 * @param[out] data Int16 array of (x,y,z) sensor values
 * @returns 0 if successful
 * @retval -1 unable to claim bus
 * @retval -2 unable to transfer data
 */
int32_t PIOS_BMA180_ReadAccels(struct pios_bma180_data * data)
{
	// To save memory use same buffer for in and out but offset by
	// a byte
	uint8_t buf[7] = {BMA_X_LSB_ADDR | 0x80,0,0,0,0,0};
	uint8_t rec[7] = {0,0,0,0,0,0};
	
	if(BMA180_ClaimBus() != 0)
		return -1;
	if(SPI_TransferBlock(dev->spi_id,&buf[0],&rec[0],7,NULL) != 0)
		return -2;
	BMA180_ReleaseBus();	
	
	//        |    MSB        |   LSB       | 0 | new_data |
	data->x = ((rec[2] << 8) | rec[1]);
	data->y = ((rec[4] << 8) | rec[3]);
	data->z = ((rec[6] << 8) | rec[5]);
	data->x /= 4;
	data->y /= 4;
	data->z /= 4;
	
	return 0; // return number of remaining entries
}

/**
 * @brief Returns the scale the BMA180 chip is using
 * @return Scale (m / s^2) / LSB
 */
float BMA180_GetScale()
{
	if(BMA180_Validate(dev) != 0)
		return -1;

	switch (dev->cfg->range) {
		case BMA_RANGE_1G:
			return GRAVITY / 8192.0f;
		case BMA_RANGE_1_5G:
			return GRAVITY / 5460.0f;
		case BMA_RANGE_2G:
			return GRAVITY / 4096.0f;
		case BMA_RANGE_3G:
			return GRAVITY / 2730.0f;
		case BMA_RANGE_4G:
			return GRAVITY / 2048.0f;
		case BMA_RANGE_8G:
			return GRAVITY / 1024.0f;
		case BMA_RANGE_16G:
			return GRAVITY / 512.0f;
	}
	return 0;
}

/**
 * @brief Get data from fifo
 * @param [out] buffer pointer to a @ref pios_bma180_data structure to receive data
 * @return 0 for success, -1 for failure (no data available)
 */
int32_t BMA180_ReadFifo(struct bma180_data * buffer)
{
	if(BMA180_Validate(dev) != 0)
		return -1;

	if(fifoBuf_getUsed(&dev->fifo) < sizeof(*buffer))
		return -1;
	
	fifoBuf_getData(&dev->fifo, (uint8_t *) buffer, sizeof(*buffer));
	
	return 0;
}


/**
 * @brief Test SPI and chip functionality by reading chip ID register
 * @return 0 if success, -1 if failure.
 *
 */
int32_t BMA180_Test()
{
	// Read chip ID then version ID
	uint8_t buf[3] = {0x80 | BMA_CHIPID_ADDR, 0, 0};
	uint8_t rec[3] = {0,0, 0};
	int32_t retval;

	if(BMA180_ClaimBus() != 0)
		return -1;
	retval = SPI_TransferBlock(dev->spi_id,&buf[0],&rec[0],sizeof(buf),NULL);
	BMA180_ReleaseBus();
	
	if(retval != 0)
		return -2;
	
	struct pios_bma180_data data;
	if(BMA180_ReadAccels(&data) != 0)
		return -3;
	
	if(rec[1] != 0x3)
		return -4;
	
	if(rec[2] < 0x12)
		return -5;

	return 0;
}

/**
 * @brief IRQ Handler.  Read data from the BMA180 FIFO and push onto a local fifo.
 */
int32_t bma180_irqs = 0;
bool_t BMA180_IRQHandler(void)
{
	bma180_irqs++;
	
	const static uint8_t bma180_req_buf[7] = {BMA_X_LSB_ADDR | 0x80,0,0,0,0,0};
	uint8_t bma180_dmabuf[8];

	// If we can't get the bus then just move on for efficiency
	bool woken = false;
	if(BMA180_ClaimBusISR(&woken) != 0) {
		return woken; // Something else is using bus, miss this data
	}
		
	SPI_TransferBlock(dev->spi_id,bma180_req_buf,(uint8_t *) bma180_dmabuf, 
							   sizeof(bma180_dmabuf), NULL);

	// TODO: Make this conversion depend on configuration scale
	struct bma180_data data;
	
	// Don't release bus till data has copied
	BMA180_ReleaseBus(&woken);
	
	// Must not return before releasing bus
	if(fifoBuf_getFree(&dev->fifo) < sizeof(data))
		return woken;
	
	// Bottom two bits indicate new data and are constant zeros.  Don't right 
	// shift because it drops sign bit
	data.x = ((bma180_dmabuf[2] << 8) | bma180_dmabuf[1]);
	data.y = ((bma180_dmabuf[4] << 8) | bma180_dmabuf[3]);
	data.z = ((bma180_dmabuf[6] << 8) | bma180_dmabuf[5]);
	data.x /= 4;
	data.y /= 4;
	data.z /= 4;
	data.temperature = bma180_dmabuf[7];
	
	fifoBuf_putData(&dev->fifo, (uint8_t *) &data, sizeof(data));
	
	return woken;
}


/**
 * @}
 * @}
 */
