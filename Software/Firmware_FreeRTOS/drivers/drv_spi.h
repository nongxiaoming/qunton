#ifndef __DRV_SPI_H
#define __DRV_SPI_H
#include "rtos.h"


/* SPI Hardware Device */
  
#define SPI_CLK_GPIO_PORT          GPIOB       
#define I2C1_SCL_GPIO_CLK          RCC_AHB1Periph_GPIOB 
#define I2C1_SCL_GPIO_PIN          GPIO_Pin_6
#define I2C1_SCL_GPIO_PINSOURCE    GPIO_PinSource6 
  
#define I2C1_SDA_GPIO_PORT         GPIOB       
#define I2C1_SDA_GPIO_CLK          RCC_AHB1Periph_GPIOB 
#define I2C1_SDA_GPIO_PIN          GPIO_Pin_7 
#define I2C1_SDA_GPIO_PINSOURCE    GPIO_PinSource7 
  
#define I2C1_CLK                   RCC_APB1Periph_I2C1
#define I2C1_AF                    GPIO_AF_I2C1  
  
#define I2C1_DMA                   DMA1
#define I2C1_DMA_CLK               RCC_AHB1Periph_DMA1 
#define I2C1_DMA_CHANNEL           DMA_Channel_1  
   
#define I2C1_IT_EVT_IRQn           I2C1_EV_IRQn  
#define I2C1_IT_ERR_IRQn           I2C1_ER_IRQn   
    
#define I2C1_DMA_TX_Stream         DMA1_Stream6
#define I2C1_DMA_TX_IRQn           DMA1_Stream6_IRQn
#define I2C1_DMA_TX_IRQHandler     DMA1_Stream6_IRQHandler
#define I2C1_DMA_TX_TC_FLAG        DMA_FLAG_TCIF6
#define I2C1_DMA_TX_HT_FLAG        DMA_FLAG_HTIF6
#define I2C1_DMA_TX_TE_FLAG        DMA_FLAG_TEIF6

#define I2C1_DMA_RX_Stream         DMA1_Stream0
#define I2C1_DMA_RX_IRQn           DMA1_Stream0_IRQn
#define I2C1_DMA_RX_IRQHandler     DMA1_Stream0_IRQHandler
#define I2C1_DMA_RX_TC_FLAG        DMA_FLAG_TCIF0
#define I2C1_DMA_RX_HT_FLAG        DMA_FLAG_HTIF0
#define I2C1_DMA_RX_TE_FLAG        DMA_FLAG_TEIF0 
  


																								 
#endif /*__DRV_I2C_H */

