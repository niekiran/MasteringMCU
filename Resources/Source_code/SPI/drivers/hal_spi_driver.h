#ifndef __HAL_SPI_DRIVER_H
#define __HAL_SPI_DRIVER_H

/* MC specific headerfile for stm32f407vgt6 based Discovery board */
#include "stm32f407xx.h"

/******************************************************************************/
/*                                                                            */
/*                        1. Serial Peripheral Interface                         */
/*                           Register Bit Defininitions                          */
/******************************************************************************/
	


/*******************  Bit definition for SPI_CR1 register  ********************/
#define SPI_REG_CR1_BIDIMODE                     	( (uint32_t) 1 << 15 )
#define SPI_ENABLE_2_LINE_UNI_DIR        					0
#define SPI_ENABLE_1_LINE_BIDI           					1

#define SPI_REG_CR1_DFF           								( ( uint32_t) 1 << 11 )
#define SPI_8BIT_DF_ENABLE  											0
#define SPI_16_BIT_DF_ENABLE 											1


#define SPI_REG_CR1_SSM           								( ( uint32_t) 1 << 9 )
#define SPI_SSM_ENABLE     												1
#define SPI_SSM_DISABLE    												0


#define SPI_REG_CR1_SSI           								( ( uint32_t) 1 << 8 )

#define SPI_CR1_LSBFRIST      										( ( uint32_t) 1 << 7 )
#define SPI_TX_MSB_FIRST        									0
#define SPI_TX_LSB_FIRST        									1

#define SPI_REG_CR1_SPE             							( ( uint32_t) 1 << 6 )

#define SPI_REG_CR1_BR_PCLK_DIV_2   							( ( uint32_t)  0 << 3 )
#define SPI_REG_CR1_BR_PCLK_DIV_4   							( ( uint32_t)  1 << 3 ) 
#define SPI_REG_CR1_BR_PCLK_DIV_8   							( ( uint32_t)  2 << 3 ) 
#define SPI_REG_CR1_BR_PCLK_DIV_16  							( ( uint32_t)  3 << 3 ) 
#define SPI_REG_CR1_BR_PCLK_DIV_32  							( ( uint32_t)  4 << 3 ) 
#define SPI_REG_CR1_BR_PCLK_DIV_64  							( ( uint32_t)  5 << 3 ) 
#define SPI_REG_CR1_BR_PCLK_DIV_128  							( ( uint32_t)  6 << 3 ) 
#define SPI_REG_CR1_BR_PCLK_DIV_256  							( ( uint32_t)  7 << 3 )

#define SPI_REG_CR1_MSTR           								( ( uint32_t) 1 << 2) 
#define SPI_MASTER_MODE_SEL      									1
#define SPI_SLAVE_MODE_SEL       									0


#define SPI_REG_CR1_CPOL           								( ( uint32_t) 1 << 1) 

#define SPI_CPOL_LOW        											0
#define SPI_CPOL_HIGH       											1

#define SPI_REG_CR1_CPHA           								( ( uint32_t) 1 << 0) 
#define SPI_FIRST_CLOCK_TRANS  										0
#define SPI_SECOND_CLOCK_TRANS 										1 


/*******************  Bit definition for SPI_CR2 register  ********************/
#define SPI_REG_CR2_TXEIE_ENABLE    							( ( uint32_t) 1 << 7) 
#define SPI_REG_CR2_RXNEIE_ENABLE    							( ( uint32_t) 1 << 6) 
#define SPI_REG_CR2_ERRIE_ENABLE    							( ( uint32_t) 1 << 5)

#define SPI_REG_CR2_FRAME_FORMAT    							( ( uint32_t) 1 << 4)
#define SPI_MOTOROLA_MODE        									0
#define SPI_TI_MODE             	 								1

#define SPI_REG_CR2_SSOE            							( ( uint32_t) 1 << 2) 


/*******************  Bit definition for SPI_SR register  ********************/
#define SPI_REG_SR_FRE_FLAG        								( ( uint32_t) 1 << 8) 
#define SPI_REG_SR_BUSY_FLAG        							( ( uint32_t) 1 << 7)
#define SPI_REG_SR_TXE_FLAG        								( ( uint32_t) 1 << 1)
#define SPI_REG_SR_RXNE_FLAG        							( ( uint32_t) 1 << 0)


/* SPI device base address */
#define SPI_1 SPI1
#define SPI_2 SPI2
#define SPI_3 SPI3

#define SPI_IS_BUSY 1 
#define SPI_IS_NOT_BUSY 0 

/* Macros to Enable Clock for diffrent SPI devices */

#define _HAL_RCC_SPI1_CLK_ENABLE()       ( RCC->APB2ENR |=  (1 << 12) )
#define _HAL_RCC_SPI2_CLK_ENABLE()       ( RCC->APB1ENR |= ( 1 << 14) )
#define _HAL_RCC_SPI3_CLK_ENABLE()       ( RCC->APB1ENR |= ( 1 << 15) )


#define  RESET  0 
#define  SET  !RESET

/******************************************************************************/
/*                                                                            */
/*                      2. Data Structures used by SPI Driver                    */
/*                                                                            */
/******************************************************************************/

/**
  * @brief  HAL SPI State structure definition
  */
typedef enum
{
  HAL_SPI_STATE_RESET      = 0x00,  /*  SPI not yet initialized or disabled                */
  HAL_SPI_STATE_READY      = 0x01,  /*  SPI initialized and ready for use                  */
  HAL_SPI_STATE_BUSY       = 0x02,  /*  SPI process is ongoing                             */
  HAL_SPI_STATE_BUSY_TX    = 0x12,  /*  Data Transmission process is ongoing               */
  HAL_SPI_STATE_BUSY_RX    = 0x22,  /*  Data Reception process is ongoing                  */
  HAL_SPI_STATE_BUSY_TX_RX = 0x32,  /*  Data Transmission and Reception process is ongoing */
  HAL_SPI_STATE_ERROR      = 0x03   /*  SPI error state                                    */
}hal_spi_state_t;

/** 
  * @brief  SPI Configuration Structure definition  
  */

typedef struct
{
  uint32_t Mode;               /*  Specifies the SPI operating mode. */
                                   

  uint32_t Direction;          /*  Specifies the SPI Directional mode state. */
                                  
  uint32_t DataSize;           /*  Specifies the SPI data size. */
                                   

  uint32_t CLKPolarity;        /*  Specifies the serial clock steady state. */
                                   

  uint32_t CLKPhase;           /*  Specifies the clock active edge for the bit capture. */
                                 

  uint32_t NSS;                /*  Specifies whether the NSS signal is managed by
                                    hardware (NSS pin) or by software using the SSI bit. */
                                  

  uint32_t BaudRatePrescaler;  /*  Specifies the Baud Rate prescaler value which will be
                                    used to configure the transmit and receive SCK clock. */
                                    

  uint32_t FirstBit;           /*  Specifies whether data transfers start from MSB or LSB bit. */
                                   
}spi_init_t;

/** 
  * @brief  SPI handle Structure definition
  */
typedef struct __spi_handle_t
{
  SPI_TypeDef                *Instance;       /* SPI registers base address */

  spi_init_t                   Init;          /* SPI communication parameters */

  uint8_t                    *pTxBuffPtr;     /* Pointer to SPI Tx transfer Buffer */

  uint16_t                   TxXferSize;      /* SPI Tx transfer size */
  
  uint16_t                   TxXferCount;     /* SPI Tx Transfer Counter */

  uint8_t                    *pRxBuffPtr;     /* Pointer to SPI Rx transfer Buffer */

  uint16_t                   RxXferSize;      /* SPI Rx transfer size */

  uint16_t                   RxXferCount;     /* SPI Rx Transfer Counter */

  hal_spi_state_t  State;                     /* SPI communication state */
	
}spi_handle_t;




/******************************************************************************/
/*                                                                            */
/*                      3. Driver exposed APIs                                   */
/*                                                                            */
/******************************************************************************/

/**
	* @brief  API used to do initialize the given SPI device
	* @param  *SPIx : Base address of the SPI  
  * @param  *buffer : pointer to the rx buffer 
  * @param  len : len of rx data
  * @retval none
	*/

void hal_spi_init(spi_handle_t *spi_handle);

/**
	* @brief  API used to do master data transmission 
	* @param  *SPIx : Base address of the SPI  
  * @param  *buffer : pointer to the tx buffer 
  * @param  len : len of tx data
  * @retval none
	*/
void hal_spi_master_tx(spi_handle_t *spi_handle,uint8_t *buffer, uint32_t len);

/**
	* @brief  API used to do slave data transmission 
	* @param  *SPIx : Base address of the SPI  
  * @param  *buffer : pointer to the tx buffer 
  * @param  len : len of tx data
  * @retval none
	*/
void hal_spi_slave_tx(spi_handle_t *spi_handle, uint8_t *rcv_buffer, uint32_t len);


/**
	* @brief  API used to do master data reception 
	* @param  *SPIx : Base address of the SPI  
  * @param  *buffer : pointer to the rx buffer 
  * @param  len : len of rx data
  * @retval none
	*/
void hal_spi_master_rx(spi_handle_t *spi_handle,uint8_t *buffer, uint32_t len);


/**
	* @brief  API used to do slave data reception 
	* @param  *SPIx : Base address of the SPI  
  * @param  *buffer : pointer to the rx buffer 
  * @param  len : len of rx data
  * @retval none
	*/
void hal_spi_slave_rx(spi_handle_t *spi_handle, uint8_t *rcv_buffer, uint32_t len);


/**
  * @brief  This function handles SPI interrupt request.
  * @param  hspi: pointer to a spi_handle_t structure that contains
  *                the configuration information for SPI module.
  * @retval none
  */
void hal_spi_irq_handler(spi_handle_t *hspi);
#endif