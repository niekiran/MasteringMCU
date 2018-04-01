#include <stdint.h>
#include "hal_spi_driver.h"
 
 
 /**
	* @brief  Enables the SPI device   
	* @param  *SPIx : Base address of the SPI  
	* @retval None
	*/
void hal_spi_enable(SPI_TypeDef *SPIx)
{
	if( !(SPIx->CR1 & SPI_REG_CR1_SPE) )
		SPIx->CR1 |= SPI_REG_CR1_SPE;
}

 /**
	* @brief  Disables the SPI device   
	* @param  *SPIx : Base address of the SPI  
	* @retval None
	*/
void hal_spi_disable(SPI_TypeDef *SPIx)
{
	SPIx->CR1 &= ~SPI_REG_CR1_SPE;
}

 /**
s	* @brief  Configures SPI clk phase and polarity   
	* @param  *SPIx : Base address of the SPI  
  * @param  phase : configures phase ,  
  * @param  polarity : configures polarity 
	* @retval None
	*/
void hal_spi_configure_phase_and_polarity(SPI_TypeDef *SPIx,uint32_t phase_value, uint32_t polarity)
{
	if(phase_value )
	{
		 SPIx->CR1 |= SPI_REG_CR1_CPHA;
	}else 
	{
		  SPIx->CR1 &= ~SPI_REG_CR1_CPHA;
	}
	
	if(polarity)
	{
		SPIx->CR1 |= SPI_REG_CR1_CPOL;
	}else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_CPOL;
	}
	
} 


 /**
	* @brief  Configures master or slave mode 
	* @param  *SPIx : Base address of the SPI   
  * @param  master : if 1 , then configured for master  
	* @retval None
	*/
void hal_spi_configure_device_mode(SPI_TypeDef *SPIx, uint32_t master)
{
	if(master)
	{
	  SPIx->CR1 |= SPI_REG_CR1_MSTR;
	}
	else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_MSTR;
	}
}


 /**
	* @brief  Configures SPI datasize 
	* @param  *SPIx : Base address of the SPI  
  * @param  datasize : data size to be configured  ,  
  * @param  lsbmsbfirst : if 1, lsb will be sent first.  
	* @retval None
	*/
void hal_spi_configure_datasize(SPI_TypeDef *SPIx, uint32_t datasize_16, uint32_t lsbfirst)
{
	if(datasize_16)
	{
		SPIx->CR1 |= SPI_REG_CR1_DFF;
	}else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_DFF;
	}
	
	if(lsbfirst)
	{
	  SPIx->CR1 |= SPI_CR1_LSBFRIST;
	}else
	{
		SPIx->CR1 &= ~SPI_CR1_LSBFRIST;
	}
}


 /**
	* @brief  Configures the NSS pin of the master 
	* @param  *SPIx : Base address of the SPI    
	* @retval None
	*/
void hal_spi_configure_nss_master(SPI_TypeDef *SPIx, uint32_t ssm_enable)
{
	
	#if 0
	if(ssm_enable)
	{
		SPIx->CR1 |=  SPI_REG_CR1_SSM ;
		SPIx->CR1 |= SPI_REG_CR1_SSI;
	}else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_SSM;
	}
	
	#endif 
	
		if(ssm_enable)
	{
		SPIx->CR1 |= ( SPI_REG_CR1_SSM | (1 << 8) );
	}else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_SSM;
	}
	
}

 /**
	* @brief  Configures the NSS pin of the slave 
	* @param  *SPIx : Base address of the SPI  
	* @retval None
	*/
void hal_spi_configure_nss_slave(SPI_TypeDef *SPIx, uint32_t ssm_enable)
{
	
	if(ssm_enable)
	{
		SPIx->CR1 |= SPI_REG_CR1_SSM ;
	}else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_SSM;
	}
	
}


 /**
	* @brief  Configures SPI baudrate
	* @param  *SPIx : Base address of the SPI  
  * @param  pre_scalar_value : pre scalar value to be used to generate baudrate 
	* @retval None
	*/
void hal_spi_configure_baudrate(SPI_TypeDef *SPIx, uint32_t pre_scalar_value)
{
        if(pre_scalar_value > 7 )
            SPIx->CR1 |= (0x00 << 3); //if pre_scalar_value > 7,then use default . that is 0
        else
            SPIx->CR1 |= (pre_scalar_value << 3);
}


 /**
	* @brief  Configures SPI direction
	* @param  *SPIx : Base address of the SPI  
  * @param  direction : if 1, direction will be single line bi-directional else, 2 lines uni directional 
	* @retval None
	*/
void hal_spi_configure_direction(SPI_TypeDef *SPIx, uint32_t direction )
{
	if(direction )
	{
		SPIx->CR1 |= SPI_REG_CR1_BIDIMODE; 
	}else
	{
		SPIx->CR1 &= ~SPI_REG_CR1_BIDIMODE; 
	}
		
}

 /**
	* @brief  Enables the Tx buffer empty interrupt (TXE)
	* @param  *SPIx : Base address of the SPI  
  * @retval None
	*/

static void hal_spi_enable_txe_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 |= SPI_REG_CR2_TXEIE_ENABLE;
}

 /**
	* @brief  Disables the Tx buffer empty interrupt (TXE)
	* @param  *SPIx : Base address of the SPI  
  * @retval None
	*/
static void hal_spi_disable_txe_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 &= ~SPI_REG_CR2_TXEIE_ENABLE;
}

 /**
	* @brief  Enables the RX buffer non empty interrupt (RXNE)
	* @param  *SPIx : Base address of the SPI  
  * @retval None
	*/
static void hal_spi_enable_rxne_interrupt(SPI_TypeDef *SPIx)
{
	
	SPIx->CR2 |= SPI_REG_CR2_RXNEIE_ENABLE;
}

/**
	* @brief  Disables the RX buffer non empty interrupt (RXNE)
	* @param  *SPIx : Base address of the SPI  
  * @retval None
	*/
static void hal_spi_disable_rxne_interrupt(SPI_TypeDef *SPIx)
{
	SPIx->CR2 &= ~SPI_REG_CR2_RXNEIE_ENABLE;
}


/**
	* @brief  Checks whether bus is free or busy 
	* @param  *SPIx : Base address of the SPI  
  * @retval return 1, if bus is busy 
	*/
uint8_t hal_spi_is_bus_busy(SPI_TypeDef *SPIx)
{
	if (SPIx->SR & SPI_REG_SR_BUSY_FLAG )
	{
		return SPI_IS_BUSY;
	}else
	   return SPI_IS_NOT_BUSY;
	
}

/**
	* @brief  API used to do master data transmission 
	* @param  *SPIx : Base address of the SPI  
  * @param  *buffer : pointer to the tx buffer 
  * @param  len : len of tx data
  * @retval none
	*/
void hal_spi_master_tx(spi_handle_t *spi_handle,uint8_t *buffer, uint32_t len)
{
	spi_handle->pTxBuffPtr = buffer;
	spi_handle->TxXferCount = len;
	spi_handle->TxXferSize = len;
	
		spi_handle->RxXferCount = 0;
	spi_handle->RxXferSize = 0 ;

	spi_handle->State = HAL_SPI_STATE_BUSY_TX;
	
	hal_spi_enable_txe_interrupt(spi_handle->Instance); 
	hal_spi_enable(spi_handle->Instance);	

	
	
}



/**
	* @brief  API used to do master data reception 
	* @param  *SPIx : Base address of the SPI  
  * @param  *buffer : pointer to the rx buffer 
  * @param  len : len of rx data
  * @retval none
	*/

uint8_t dummu_tx[2]={0};
void hal_spi_master_rx(spi_handle_t *spi_handle,uint8_t *rx_buffer, uint32_t len)
{
	uint32_t i =0,val;

	
/* this is a dummy tx */
	spi_handle->pTxBuffPtr   = dummu_tx;
	spi_handle->TxXferSize   = len;
	spi_handle->TxXferCount  = len;

/* data will be rxed to rx_buffer */
	spi_handle->pRxBuffPtr   = rx_buffer;
	spi_handle->RxXferSize   = len;
	spi_handle->RxXferCount  = len;

	/* Driver is busy in RX */
	spi_handle->State = HAL_SPI_STATE_BUSY_RX;

	

	/* read data register once before enabling 
	 * the RXNE interrupt to make sure DR is empty
	 */
	
	val = spi_handle->Instance->DR;
	
	/* Now enable both TXE and RXNE Interrupt */
	hal_spi_enable_rxne_interrupt(spi_handle->Instance);
	hal_spi_enable_txe_interrupt(spi_handle->Instance);
	
	hal_spi_enable(spi_handle->Instance);

}

uint8_t dummy_rx[10]={0};

/**
	* @brief  API used to do slave data transmission 
	* @param  *SPIx : Base address of the SPI  
  * @param  *buffer : pointer to the tx buffer 
  * @param  len : len of tx data
  * @retval none
	*/
void hal_spi_slave_tx(spi_handle_t *spi_handle, uint8_t *tx_buffer, uint32_t len)
{
	

	/* populate the pointers and lenght information to TX the data */
	spi_handle->pTxBuffPtr   = tx_buffer;
	spi_handle->TxXferSize   = len;
	spi_handle->TxXferCount  = len;

	/* pointers to handle dummy rx, you can reuse the same pointer */
	spi_handle->pRxBuffPtr = dummy_rx;
	spi_handle->RxXferSize   = len;
	spi_handle->RxXferCount  = len;
	
	/* Driver is busy in doing TX */
	spi_handle->State        = HAL_SPI_STATE_BUSY_TX;

	

	/* Now enable both TXE and RXNE Interrupt */
	hal_spi_enable_rxne_interrupt(spi_handle->Instance);
	hal_spi_enable_txe_interrupt(spi_handle->Instance); 
	
	hal_spi_enable(spi_handle->Instance);
	
}

/**
	* @brief  API used to do slave data reception 
	* @param  *SPIx : Base address of the SPI  
  * @param  *buffer : pointer to the rx buffer 
  * @param  len : len of rx data
  * @retval none
	*/

void hal_spi_slave_rx(spi_handle_t *spi_handle, uint8_t *rcv_buffer, uint32_t len)
{
	/* Populate the rcv buffer pointer address along with size in the handle */
	spi_handle->pRxBuffPtr   = rcv_buffer;
	spi_handle->RxXferSize   = len;
	spi_handle->RxXferCount  = len ; 
	
	/* Driver is busy in RX */
	spi_handle->State        = HAL_SPI_STATE_BUSY_RX;

	

	/*slave need to rcv data, so enable the RXNE interrupt */
	/*Byte reception will be taken care in the RXNE Interrupt handling code */
	hal_spi_enable_rxne_interrupt(spi_handle->Instance);
	
	/*enable the peripheral , if its not enabled */
	hal_spi_enable(spi_handle->Instance);	
}

/**
	* @brief  API used to do initialize the given SPI device
	* @param  *SPIx : Base address of the SPI  
  * @param  *buffer : pointer to the rx buffer 
  * @param  len : len of rx data
  * @retval none
	*/

void hal_spi_init(spi_handle_t *spi_handle)
{
	/* configure the phase and polartiy */
	hal_spi_configure_phase_and_polarity(spi_handle->Instance, \
	spi_handle->Init.CLKPhase, spi_handle->Init.CLKPolarity );
	
	/* Configure the spi device mode */
	hal_spi_configure_device_mode(spi_handle->Instance,spi_handle->Init.Mode );
	
	/* Configure the spi data size */
	hal_spi_configure_datasize(spi_handle->Instance, spi_handle->Init.DataSize,spi_handle->Init.FirstBit);
	
	/* Configure the slave select line */
	if(spi_handle->Init.Mode == SPI_MASTER_MODE_SEL)
	hal_spi_configure_nss_master(spi_handle->Instance,spi_handle->Init.NSS);
	else
	hal_spi_configure_nss_slave(spi_handle->Instance,spi_handle->Init.NSS);

	/* Configure the  SPI deivce speed */
	hal_spi_configure_baudrate(spi_handle->Instance,spi_handle->Init.BaudRatePrescaler);
	
	/* Configure the SPI device direction */
	hal_spi_configure_direction(spi_handle->Instance,spi_handle->Init.Direction);
	
}

/**
  * @}
  */

  /**
  * @brief   close Tx transfer 
  * @param  hspi: pointer to a spi_handle_t structure that contains
  *                the configuration information for SPI module.
  * @retval void
  */
static void hal_spi_tx_close_interrupt(spi_handle_t *hspi)
{
	
  /* Disable TXE interrupt */
  hal_spi_disable_txe_interrupt(hspi->Instance);
	
	/* if master and if driver state is not HAL_SPI_STATE_BUSY_RX then make state = READY */
	if (hspi->Init.Mode && (hspi->State != HAL_SPI_STATE_BUSY_RX))
		hspi->State = HAL_SPI_STATE_READY;

}
/**
  * @brief   handles  TXE interrupt  
  * @param  hspi: pointer to a spi_handle_t structure that contains
  *               the configuration information for SPI module.
  * @retval void
  */
 void hal_spi_handle_tx_interrupt(spi_handle_t *hspi)
{
  /* Transmit data in 8 Bit mode */
  if(hspi->Init.DataSize == SPI_8BIT_DF_ENABLE)
  {
    hspi->Instance->DR = (*hspi->pTxBuffPtr++);
		 hspi->TxXferCount--; //we sent 1 byte
  }
  /* Transmit data in 16 Bit mode */
  else
  {
    hspi->Instance->DR = *((uint16_t*)hspi->pTxBuffPtr);
    hspi->pTxBuffPtr+=2;
		hspi->TxXferCount-=2; //we sent 2 bytes in one go
  }
	
  if(hspi->TxXferCount == 0)
  {
		/* we reached end of transmission, so close the txe interrupt */
    hal_spi_tx_close_interrupt(hspi);
  }
}



/**
  * @brief   close Rx transfer 
  * @param  hspi: pointer to a spi_handle_t structure that contains
  *                the configuration information for SPI module.
  * @retval void
  */
static void hal_spi_close_rx_interrupt(spi_handle_t *hspi)
{
	while(hal_spi_is_bus_busy(hspi->Instance));
  /* Disable RXNE interrupt */
 
	hal_spi_disable_rxne_interrupt(hspi->Instance);
   hspi->State = HAL_SPI_STATE_READY;
}

/**
  * @brief   handles  RXNE interrupt
  * @param  hspi: pointer to a spi_handle_t structure that contains
  *                the configuration information for SPI module.
  * @retval void
  */
static void hal_spi_handle_rx_interrupt(spi_handle_t *hspi)
{
  /* Receive data in 8 Bit mode */
  if(hspi->Init.DataSize == SPI_8BIT_DF_ENABLE)
  {
		//NULL check
		if(hspi->pRxBuffPtr)
			(*hspi->pRxBuffPtr++) = hspi->Instance->DR;
		hspi->RxXferCount--;
  }
  /* Receive data in 16 Bit mode */
  else
  {
    *((uint16_t*)hspi->pRxBuffPtr) = hspi->Instance->DR;
    hspi->pRxBuffPtr+=2;
		hspi->RxXferCount-=2;
  }
    
  if(hspi->RxXferCount == 0)
  {
		/*We are done with the Rxing of data, lets close the rxne interrupt */
    hal_spi_close_rx_interrupt(hspi);
  }
}
/**
  * @brief  This function handles SPI interrupt request.
  * @param  hspi: pointer to a spi_handle_t structure that contains
  *                the configuration information for SPI module.
  * @retval none
  */
void hal_spi_irq_handler(spi_handle_t *hspi)
{
  uint32_t tmp1 = 0, tmp2 = 0;
	
	/* check to see RXNE is set in the status register */
	tmp1 = (hspi->Instance->SR & SPI_REG_SR_RXNE_FLAG);
	/* check whether RXNEIE bit is enabled in the control register. */
	tmp2 = (hspi->Instance->CR2 & SPI_REG_CR2_RXNEIE_ENABLE);

 
  if((tmp1 != RESET) && (tmp2 != RESET) )
  {
		/* RXNE flag is set
		 * handle the RX of data bytes
		 */
		 hal_spi_handle_rx_interrupt(hspi);
			
		return;
  } 

	/* check to see TXE is set in the status register */
	tmp1 = (hspi->Instance->SR & SPI_REG_SR_TXE_FLAG);
	tmp2 = (hspi->Instance->CR2 & SPI_REG_CR2_TXEIE_ENABLE);
 
  if((tmp1 != RESET) && (tmp2 != RESET))
  {
		/* TXE flag is set
	   * handle the TX of data bytes
	   */
     hal_spi_handle_tx_interrupt(hspi);
    return;
  }

}

