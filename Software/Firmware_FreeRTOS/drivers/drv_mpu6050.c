#include "drv_mpu6050.h"
#include "drv_usart.h"

uint8_t MPU6050_Buffer[14] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 


__IO uint32_t  MPU6050_Timeout = 50000; 


static uint32_t MPU6050_Status (void)
{

  return I2C1_IsDeviceReady(MPU6050_DEFAULT_ADDRESS);
}

/**
  * @brief  Checks the MPU6050 status.
  * @param  None
  * @retval ErrorStatus: MPU6050 Status (ERROR or SUCCESS).
  */
ErrorStatus MPU6050_GetStatus(void)
{  
	MPU6050_Timeout = 50000;
  /* Test if MPU6050 is ready */
  while ((MPU6050_Status() == SUCCESS) && MPU6050_Timeout)  
  {
    MPU6050_Timeout--;
  }
  
  /* If MPU6050 is not responding return ERROR */
  if (MPU6050_Timeout == 0)
  {
    return ERROR;
  }
  
  /* In other case return SUCCESS */
  return SUCCESS;  
}
/**
  * @brief  Read the specified register from the MPU6050.
  * @param  RegName: specifies the MPU6050 register to be read.
  *              This member can be one of the following values:  
  *                  - MPU6050_REG_TEMP: temperature register
  *                  - MPU6050_REG_TOS: Over-limit temperature register
  *                  - MPU6050_REG_THYS: Hysteresis temperature register
  * @retval MPU6050 register value.
  */
uint8_t MPU6050_ReadReg(uint8_t RegName)
{   
  uint8_t tmp = 0;
  
  MPU6050_Buffer[0] = 0;
  MPU6050_Buffer[1] = 0;
  
  /* Read Operation */
  if(I2C1_Read(MPU6050_DEFAULT_ADDRESS,RegName,MPU6050_Buffer,1) == SUCCESS)
  {
    /* Store MPU6050_I2C received data */
  tmp = MPU6050_Buffer[0];
  }
 
  /* return a Reg value */
  return tmp;  
}

/**
  * @brief  Write to the specified register of the MPU6050.
  * @param  RegName: specifies the MPU6050 register to be written.
  *              This member can be one of the following values:    
  *                  - MPU6050_REG_TOS: Over-limit temperature register
  *                  - MPU6050_REG_THYS: Hysteresis temperature register
  * @param  RegValue: value to be written to MPU6050 register.  
  * @retval None
  */
ErrorStatus MPU6050_WriteReg(uint8_t RegName, uint8_t RegValue)
{   
  MPU6050_Buffer[0] = RegValue;
     
  return I2C1_Write(MPU6050_DEFAULT_ADDRESS,RegName,MPU6050_Buffer,1);
}
/**************************实现函数********************************************
*函数原型:		u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的1个位
输入	dev  目标设备地址
reg	   寄存器地址
bitNum  要修改目标字节的bitNum位
data  为0 时，目标位将被清0 否则将被置位
返回   成功 为1 
失败为0
*******************************************************************************/ 
void IICwriteBit(uint8_t reg, uint8_t bitNum, uint8_t data){
	uint8_t b;
	b=MPU6050_ReadReg(reg);
	b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
	MPU6050_WriteReg(reg,b);
}
/**************************实现函数********************************************
*函数原型:		u8 IICwriteBits(u8 dev,u8 reg,u8 bitStart,u8 length,u8 data)
*功　　能:	    读 修改 写 指定设备 指定寄存器一个字节 中的多个位
输入	dev  目标设备地址
reg	   寄存器地址
bitStart  目标字节的起始位
length   位长度
data    存放改变目标字节位的值
返回   成功 为1 
失败为0
*******************************************************************************/ 
void IICwriteBits(u8 reg,u8 bitStart,u8 length,u8 data)
{
	
	u8 b,mask;
	b=MPU6050_ReadReg(reg);
	mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
	data <<= (8 - length);
	data >>= (7 - bitStart);
	b &= mask;
	b |= data;
	MPU6050_WriteReg(reg,b);
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
	IICwriteBits(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
	
}
/** Set full-scale gyroscope range.
* @param range New full-scale gyroscope range value
* @see getFullScaleRange()
* @see MPU6050_GYRO_FS_250
* @see MPU6050_RA_GYRO_CONFIG
* @see MPU6050_GCONFIG_FS_SEL_BIT
* @see MPU6050_GCONFIG_FS_SEL_LENGTH
*/
void MPU6050_setFullScaleGyroRange(uint8_t range) {
	IICwriteBits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
	IICwriteBits(MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}
/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
enabled =1   睡觉
enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
	IICwriteBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
	IICwriteBit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
	IICwriteBit(MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

void MPU6050_setDLPF(uint8_t mode)
{
	IICwriteBits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}


uint8_t MPU6050ReadID(void)
{

  MPU6050_Buffer[0] = 0;
  MPU6050_Buffer[1] = 0;
  
  /* Read Operation */
  if(I2C1_Read(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_WHO_AM_I,MPU6050_Buffer,1) == SUCCESS)
  {
  }

    return MPU6050_Buffer[0] ;
}
void MPU6050ReadData(short *Data)
{

  /* Read Operation */
  if(I2C1_Read(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,MPU6050_Buffer,14) == SUCCESS)
  {
    Data[0] = (MPU6050_Buffer[0] << 8) | MPU6050_Buffer[1];
    Data[1] = (MPU6050_Buffer[2] << 8) | MPU6050_Buffer[3];
    Data[2] = (MPU6050_Buffer[4] << 8) | MPU6050_Buffer[5];
	  Data[3] = (MPU6050_Buffer[6] << 8) | MPU6050_Buffer[7];
	  Data[4] = (MPU6050_Buffer[8] << 8) | MPU6050_Buffer[9];
    Data[5] = (MPU6050_Buffer[10] << 8) | MPU6050_Buffer[11];
    Data[6] = (MPU6050_Buffer[12] << 8) | MPU6050_Buffer[13];
  }

}
void MPU6050ReadGyro(short *gyroData)
{
  /* Read Operation */
  if(I2C1_Read(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_GYRO_ZOUT_H,MPU6050_Buffer,2) == SUCCESS)
  {
		gyroData[0] = (MPU6050_Buffer[0] << 8) | MPU6050_Buffer[1];
    gyroData[1] = (MPU6050_Buffer[2] << 8) | MPU6050_Buffer[3];
    gyroData[2] = (MPU6050_Buffer[4] << 8) | MPU6050_Buffer[5];
  }

}
/**
  * @brief  Initializes the MPU6050_I2C.
  * @param  None
  * @retval None
  */
void MPU6050_Init(void)
{
		u8 id=0;
  /* Initialize i2c peripheral */
	//I2CDev_StructInit(&i2c1_dev);	
  I2C1_Init();

	while(id!=0x68)
		{
			id=MPU6050ReadID();
			kprintf("MPU6050 ID:%x",id);
		}
	MPU6050_setSleepEnabled(0); //进入工作状态
	//Delay_ms_mpu(200);
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO); //设置时钟  0x6b   0x01
	//Delay_ms_mpu(200);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-2000度每秒
	//Delay_ms_mpu(50);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_4);	//加速度度最大量程 +-4G
	//Delay_ms_mpu(50);
	MPU6050_setDLPF(MPU6050_DLPF_BW_42);
	//Delay_ms_mpu(50);
	MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
	//Delay_ms_mpu(50);
	MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
	//Delay_ms_mpu(50);
}
	/* Priorities for the mpu6050 test tasks. */
#define MPU_TEST_TASK_PRIORITY				(4)
#define MPU_TEST_TASK_STACK_SIZE		 ( configMINIMAL_STACK_SIZE*64 )
static void Task_Entry(void* parameter)
{
short temp[7];
uint8_t id=0;
	MPU6050_Init();
  while(1)
  {
	  id=MPU6050ReadID();
		kprintf("id=0x%X\r\n",id);
	  MPU6050ReadData(temp);
		kprintf("data:%d,%d,%d,%d,%d,%d,%d\r\n",temp[0],temp[1],temp[2],temp[3],temp[4],temp[5],temp[6]);
		//rt_thread_delay(50);
		vTaskDelay(1);
  }
}
int MPU6050_Test_Init(void)
{

   /* Start the mpu6050 test tasks . */
    xTaskCreate( Task_Entry, "mpu6050", MPU_TEST_TASK_STACK_SIZE, NULL, MPU_TEST_TASK_PRIORITY, NULL );

    return 0;
}

