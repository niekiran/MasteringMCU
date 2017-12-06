#ifndef __HAL_I2C_DRIVER_H
#define __HAL_I2C_DRIVER_H

#include "stm32f407xx.h"
#include  <stdint.h>


/* I2C device base address */
#define I2C_1 I2C1
#define I2C_2 I2C2
#define I2C_3 I2C3

/* Macros to Enable Clock for diffrent I2C devices */

#define _HAL_RCC_I2C1_CLK_ENABLE()       ( RCC->APB1ENR |=  (1 << 21) )
#define _HAL_RCC_I2C2_CLK_ENABLE()       ( RCC->APB1ENR |= ( 1 << 22) )
#define _HAL_RCC_I2C3_CLK_ENABLE()       ( RCC->APB1ENR |= ( 1 << 23) )


/******************************************************************************/
/*                                                                            */
/*                                I2C                                         */
/*                        Register Bit Defininitions                          */
/******************************************************************************/

/*******************  Bit definition for I2C_CR1 register  ********************/

#define I2C_REG_CR1_POS      ((uint32_t)1 << 11)

#define I2C_REG_CR1_ACK      ((uint32_t)1 << 10)
#define I2C_ACK_ENABLE        1
#define I2C_ACK_DISABLE        0

#define I2C_REG_CR1_STOP_GEN               ((uint32_t)1 << 9)
#define I2C_REG_CR1_START_GEN            ((uint32_t)1 << 8)

#define I2C_REG_CR1_NOSTRETCH            ((uint32_t)1 << 7)
#define I2C_ENABLE_CLK_STRETCH   0
#define I2C_DISABLE_CLK_STRETCH  1

#define I2C_REG_CR1_ENABLE_I2C             ((uint32_t)1 << 0)

/*******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_REG_CR2_BUF_INT_ENABLE       ((uint32_t) ( 1 << 10 ) )
#define I2C_REG_CR2_EVT_INT_ENABLE       ((uint32_t) ( 1 << 9 ) )
#define I2C_REG_CR2_ERR_INT_ENABLE       ((uint32_t) ( 1 << 8 ) )

#define I2C_PERIPHERAL_CLK_FREQ_2MHZ      ((uint32_t)2 )  
#define I2C_PERIPHERAL_CLK_FREQ_3MHZ      ((uint32_t)3 )  
#define I2C_PERIPHERAL_CLK_FREQ_4MHZ     ((uint32_t)4 )  
#define I2C_PERIPHERAL_CLK_FREQ_5MHZ     ((uint32_t)5 )  
#define I2C_PERIPHERAL_CLK_FREQ_6MHZ     ((uint32_t)6 )  
#define I2C_PERIPHERAL_CLK_FREQ_7MHZ     ((uint32_t)7 )  
#define I2C_PERIPHERAL_CLK_FREQ_8MHZ     ((uint32_t)8 ) 
#define I2C_PERIPHERAL_CLK_FREQ_9MHZ     ((uint32_t)9 )  
#define I2C_PERIPHERAL_CLK_FREQ_10MHZ     ((uint32_t)10 )  

/*******************  Bit definition for I2C_OAR1 register  ********************/
#define I2C_REG_OAR1_ADDRMODE        ((uint32_t) 1 << 15 )
#define I2C_ADDRMODE_7BIT          0
#define I2C_ADDRMODE_10BI          1

#define I2C_REG_OAR1_14TH_BIT              ((uint32_t) 1 << 14 )
#define I2C_REG_OAR1_7BIT_ADDRESS_POS       1

/*******************  Bit definition for I2C_SR1 register  ********************/
#define I2C_REG_SR1_TIMEOUT_FLAG              ( (uint32_t) 1 << 14)
#define I2C_REG_SR1_OVR_FLAG              ( (uint32_t) 1 << 11)
#define I2C_REG_SR1_AF_FAILURE_FLAG            ( (uint32_t) 1 << 10)
#define I2C_REG_SR1_ARLO_FLAG            ( (uint32_t) 1 << 9)
#define I2C_REG_SR1_BUS_ERROR_FLAG            ( (uint32_t) 1 << 8)
#define I2C_REG_SR1_TXE_FLAG                  ( (uint32_t) 1 << 7)
#define I2C_REG_SR1_RXNE_FLAG                ( (uint32_t) 1 << 6)
#define I2C_REG_SR1_STOP_DETECTION_FLAG       ( (uint32_t) 1 << 4) /*  for slave */
#define I2C_REG_SR1_BTF_FLAG                 ( (uint32_t) 1 << 2)
#define I2C_REG_SR1_ADDR_FLAG                 ( (uint32_t) 1 << 1)	
#define I2C_REG_SR1_ADDR_SENT_FLAG         ( (uint32_t)1 << 1 )   //For master 
#define I2C_REG_SR1_ADDR_MATCHED_FLAG           ( (uint32_t)1 << 1 ) //For SLAVE 	
#define I2C_REG_SR1_SB_FLAG                 ( (uint32_t) 1 << 0)	

/*******************  Bit definition for I2C_SR2 register  ********************/
#define I2C_REG_SR2_BUS_BUSY_FLAG             ( (uint32_t) 1 << 1)	
#define I2C_BUS_IS_BUSY                       1
#define I2C_BUS_IS_FREE                       0

#define I2C_REG_SR2_MSL_FLAG           ( (uint32_t) 1 << 0)	
#define I2C_MASTER_MODE                1
#define I2C_SLAVE_MODE                 0

#define I2C_REG_SR2_TRA_FLAG          ( (uint32_t) 1 << 2)	
#define I2C_RX_MODE                   0
#define I2C_TX_MODE                   1

/*******************  Bit definition for I2C_CCR register  ********************/
#define I2C_REG_CCR_ENABLE_FM             ( (uint32_t) 1 << 15)
#define I2C_ENABLE_SM                     0 
#define I2C_ENABLE_FM                     1

#define I2C_REG_CCR_DUTY                   ( (uint32_t) 1 << 14)
#define I2C_FM_DUTY_16BY9                  1
#define I2C_FM_DUTY_2                      0

/******************************************************************************/
/*                                                                            */
/*                      Data Structures used by I2C Driver                    */
/*                                                                            */
/******************************************************************************/

/**
  * @brief  HAL I2C State structure definition
  */
typedef enum
{
  HAL_I2C_STATE_RESET             = 0x00,  /*!< I2C not yet initialized or disabled         */
  HAL_I2C_STATE_READY             = 0x01,  /*!< I2C initialized and ready for use           */
  HAL_I2C_STATE_BUSY              = 0x02,  /*!< I2C internal process is ongoing             */
  HAL_I2C_STATE_BUSY_TX           = 0x12,  /*!< Data Transmission process is ongoing        */
  HAL_I2C_STATE_BUSY_RX           = 0x22,  /*!< Data Reception process is ongoing           */
  HAL_I2C_STATE_MEM_BUSY_TX       = 0x32,  /*!< Memory Data Transmission process is ongoing */
  HAL_I2C_STATE_MEM_BUSY_RX       = 0x42,  /*!< Memory Data Reception process is ongoing    */
  HAL_I2C_STATE_TIMEOUT           = 0x03,  /*!< I2C timeout state                           */
  HAL_I2C_STATE_ERROR             = 0x04   /*!< I2C error state                             */
}hal_i2c_state_t;

/**
  * @brief  I2C Configuration Structure definition
  */
typedef struct
{
	uint32_t ClockSpeed;       /*!< Specifies the clock frequency.
													This parameter must be set to a value lower than 400kHz */

	uint32_t DutyCycle;        /*!< Specifies the I2C fast mode duty cycle.
													This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */

	uint32_t OwnAddress1;      /*!< Specifies the first device own address.
													This parameter can be a 7-bit or 10-bit address. */

	uint32_t AddressingMode;   /*!< Specifies if 7-bit or 10-bit addressing mode is selected.
													This parameter can be a value of @ref I2C_addressing_mode */

	uint32_t DualAddressMode;  /*!< Specifies if dual addressing mode is selected.
													This parameter can be a value of @ref I2C_dual_addressing_mode */

	uint32_t OwnAddress2;      /*!< Specifies the second device own address if dual addressing mode is selected
													This parameter can be a 7-bit address. */

	uint32_t GeneralCallMode;  /*!< Specifies if general call mode is selected.
													This parameter can be a value of @ref I2C_general_call_addressing_mode */

	uint32_t NoStretchMode;    /*!< Specifies if nostretch mode is selected.
													This parameter can be a value of @ref I2C_nostretch_mode */
	uint32_t ack_enable; 

	uint8_t master;
	
}i2c_init_t;


/** 
  * @brief  I2C handle Structure definition
  */

typedef struct
{
	I2C_TypeDef                *Instance;  /*!< I2C registers base address     */

	i2c_init_t                  Init;       /*!< I2C communication parameters   */

	uint8_t                    *pBuffPtr;  /*!< Pointer to I2C transfer buffer */

	uint32_t                   XferSize;   /*!< I2C transfer size              */

	__IO uint32_t              XferCount;  /*!< I2C transfer counter           */

	hal_i2c_state_t            State;      /*!< I2C communication state        */
	uint32_t ErrorCode;     

}i2c_handle_t;

#define  RESET  0 
#define  SET  !RESET

/*
Sm mode or SMBus:
Thigh = CCR * TPCLK1
Tlow = CCR * TPCLK1 
TPCLK1  = 1/FREQR
Thigh = (1/sm_mode_freq ) / 2
so caclulate CCR 
*/ 


/** 
  * @brief  HAL Status structures definition  
  */  
typedef enum 
{
  HAL_OK       = 0x00,
  HAL_ERROR    = 0x01,
  HAL_BUSY     = 0x02,
  HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

#define UNUSED(x) ((void)(x))

/******************************************************************************/
/*                                                                            */
/*                      Driver exposed APIs                                   */
/*                                                                            */
/******************************************************************************/


void hal_i2c_init(i2c_handle_t *handle);

void hal_i2c_manage_ack(I2C_TypeDef *i2cx, uint32_t ack_noack);

void hal_i2c_master_tx(i2c_handle_t *handle, uint8_t slave_address, uint8_t *buffer, uint32_t len);
void hal_i2c_master_rx(i2c_handle_t *handle, uint8_t slave_adr, uint8_t *buffer, uint32_t len);

void hal_i2c_slave_tx(i2c_handle_t *handle, uint8_t *buffer, uint32_t len);
void hal_i2c_slave_rx(i2c_handle_t *handle, uint8_t *buffer, uint32_t len);

void HAL_I2C_EV_IRQHandler(i2c_handle_t *hi2c);
void HAL_I2C_ER_IRQHandler(i2c_handle_t *hi2c);

#endif 