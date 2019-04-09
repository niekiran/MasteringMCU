#include<stdint.h>
#include "stm32f4xx.h"
#include "hal_spi_driver.h"
#include "hal_gpio_driver.h"
#include "spi_main.h"
#include "led.h"


spi_handle_t SpiHandle;

int TestReady = 0;

uint8_t master_write_data[4]={ 0xa, 0xb, 0xc, 0xd};

uint8_t slave_tx_buffer[4]={ 0x55, 0xaa, 0x55, 0xaa};
uint8_t slave_rx_buffer[4];

/* configure gpio for spi functionality */
void spi_gpio_init(void)
{
	gpio_pin_conf_t spi_conf;
	
	
	_HAL_RCC_GPIOB_CLK_ENABLE();
	
	/* configure GPIOB_PIN_13 for SPI CLK functionality */ 
	spi_conf.pin = SPI_CLK_PIN;
	spi_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	spi_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	spi_conf.pull = GPIO_PIN_PULL_DOWN;
	spi_conf.speed = GPIO_PIN_SPEED_MEDIUM;
	
	hal_gpio_set_alt_function(GPIOB,SPI_CLK_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
	/* configure GPIOB_PIN_14 for SPI MISO functionality */ 
	spi_conf.pin = SPI_MISO_PIN;
	spi_conf.pull = GPIO_PIN_PULL_UP;
	hal_gpio_set_alt_function(GPIOB,SPI_MISO_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
	/* configure GPIOB_PIN_15 for SPI MOSI functionality */ 
	spi_conf.pin = SPI_MOSI_PIN;
	spi_conf.pull = GPIO_PIN_PULL_UP;
	hal_gpio_set_alt_function(GPIOB,SPI_MOSI_PIN,GPIO_PIN_AF5_SPI2);
	hal_gpio_init(GPIOB, &spi_conf);
	
}

static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }
	return 0;
}
int main(void)
{
	uint16_t ack_bytes = SPI_ACK_BYTES;
	uint8_t rcv_cmd[2];
		uint8_t ack_buf[2] = {0XD5, 0XE5};
	uint16_t master_cmd;
	
	/* configure the gpios for SPI functionality */
	spi_gpio_init();
	
	/* To use LED */
	led_init();
	
	/* enable the clock for the SPI2 */
	_HAL_RCC_SPI2_CLK_ENABLE() ;
	
	/*fill up the handle structure */
	SpiHandle.Instance               = SPI_2;
	SpiHandle.Init.BaudRatePrescaler = SPI_REG_CR1_BR_PCLK_DIV_32;
	SpiHandle.Init.Direction         = SPI_ENABLE_2_LINE_UNI_DIR;
	SpiHandle.Init.CLKPhase          = SPI_SECOND_CLOCK_TRANS;
	SpiHandle.Init.CLKPolarity       = SPI_CPOL_LOW;
	SpiHandle.Init.DataSize          = SPI_8BIT_DF_ENABLE;
	SpiHandle.Init.FirstBit          = SPI_TX_MSB_FIRST;
	SpiHandle.Init.NSS               = SPI_SSM_ENABLE;
	SpiHandle.Init.Mode              = SPI_SLAVE_MODE_SEL;
	
	SpiHandle.State = HAL_SPI_STATE_READY;
	
	
	/* Call driver API to initialize the SPI device */
	hal_spi_init(&SpiHandle);
	
		/* Enable the IRQs in the NVIC */
  NVIC_EnableIRQ(SPI2_IRQn);
	

while(1)
{
	/*Make sure that driver state is ready */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* Receive the master command first */
	hal_spi_slave_rx(&SpiHandle, rcv_cmd,CMD_LENGTH );
	
	/* wait until driver finishes RXing and state becomes ready again */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* This is the command slave got */
	master_cmd = (uint16_t)(rcv_cmd[1] << 8 | rcv_cmd[0]);
	
	/* is it a valid command ? */
	if(master_cmd == CMD_MASTER_WRITE || master_cmd == CMD_MASTER_READ )
	{ 
    /* yes, send out the ACK bytes */		
		hal_spi_slave_tx(&SpiHandle, (uint8_t *)&ack_buf, ACK_LEN);
		while(SpiHandle.State != HAL_SPI_STATE_READY );
		
  }else
	{
		/*  No, Error !*/
		led_toggle(GPIOD,LED_RED);
	}
		
/* is it a write command from master ? */		
if(master_cmd == CMD_MASTER_WRITE)
{
	/* master wants to write, so get ready to receive the data */
	hal_spi_slave_rx(&SpiHandle, slave_rx_buffer,DATA_LENGTH);
	
	/* wait until the recepion completes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );

	/* compare the received data with the expected data */
	if(Buffercmp(master_write_data,slave_rx_buffer,4))
	{
		/* doesnt match Error ! */
		led_toggle(GPIOD,LED_RED);
	}else
	{
		/*matches , tOggle the blue LED */
		led_toggle(GPIOD,LED_BLUE);
	}
}

/*  or, is it a read command from master ?*/
	if(master_cmd == CMD_MASTER_READ)
	{
		/* master wants to read, so transmit data to master  */
		hal_spi_slave_tx(&SpiHandle, slave_tx_buffer, DATA_LENGTH);
		
		/* hang on , till the TXing finishes */
		while(SpiHandle.State != HAL_SPI_STATE_READY );
	}
	
} 
	return 0;
}

/**
  * @brief  This function handles SPI2 interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
	/* call the driver api to process this interrupt */
  hal_spi_irq_handler(&SpiHandle);
}	 	