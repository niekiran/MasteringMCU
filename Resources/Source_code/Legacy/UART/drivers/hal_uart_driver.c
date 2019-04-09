
#include "hal_uart_driver.h"
#include "hal_gpio_driver.h"
#include "stm32f4xx.h"
#include "led.h"


/******************************************************************************/
/*                                                                            */
/*                      Helper functions                                      */
/*                                                                            */
/******************************************************************************/

/**
	* @brief  Enable the given USART peripheral 
	* @param  *uartx : base address of the USART or UART peripheral
	* @retval None
	*/
void hal_uart_enable(USART_TypeDef *uartx)
{
	
	uartx->CR1 |= USART_REG_CR1_USART_EN;
}

/**
	* @brief  Disable the given USART peripheral 
	* @param  *uartx : base address of the USART or UART peripheral
	* @retval None
	*/
void hal_uart_disable(USART_TypeDef *uartx)
{
	uartx->CR1 &= ~USART_REG_CR1_USART_EN;
}

/**
	* @brief  Enable/Disable the Transmit block of the  given USART peripheral 
	* @param  *uartx : base address of the USART or UART peripheral
  * @param  te    : if te=1, then enable the Transmit block.
	* @retval None
	*/
void hal_uart_enable_disable_tx(USART_TypeDef *uartx, uint32_t te)
{
	if(te & USART_REG_CR1_TE)
	{
		uartx->CR1 |= USART_REG_CR1_TE;
	}else
	{
		uartx->CR1 &= ~USART_REG_CR1_TE;
	}
}

/**
	* @brief  Enable/Disable the Receive block of the  given USART peripheral 
	* @param  *uartx : base address of the USART or UART peripheral
  * @param  te    : if re=1, then enable the Receive block.
	* @retval None
	*/
void hal_uart_enable_disable_rx(USART_TypeDef *uartx, uint32_t re)
{
	if(re & USART_REG_CR1_RE)
	{
		uartx->CR1 |= USART_REG_CR1_RE;
	}else
	{
		uartx->CR1 &= ~USART_REG_CR1_RE;
	}
	
}

/**
	* @brief  Configures the word length for data transmission and reception 
	* @param  *uartx : base address of the USART or UART peripheral
  * @param  wl    : if wl=1, then word length = 1s,9bits,nstop
	* @retval None
	*/
void hal_uart_configure_word_length(USART_TypeDef *uartx, uint32_t wl)
{
	if(wl)
	{
		uartx->CR1 |= USART_REG_CR1_UART_WL;//9data bits
	}else
	{
		uartx->CR1 &= ~USART_REG_CR1_UART_WL;//8 data bits 
	}
	
}

/**
	* @brief  Configures the number of stop bits 
	* @param  *uartx : base address of the USART or UART peripheral
  * @param  nstop    : this value configures the stop bits
	* @retval None
	*/
void hal_uart_configure_stop_bits(USART_TypeDef *uartx, uint32_t nstop)
{
	//clear the 12th and 13 th bits 
	uartx->CR2 &= ~( 0x3 << USART_REG_CR2_STOP_BITS);
	
	if(nstop == USART_STOP_BITS_HALF)
	{
		uartx->CR2 |= ( 0x01<< USART_REG_CR2_STOP_BITS);//0.5 stop bits
	
	}else if(nstop == USART_STOP_BITS_2)
	{
		uartx->CR2 |= ( 0x02<< USART_REG_CR2_STOP_BITS);//2 stop bits 
	
	}else if (nstop == USART_STOP_BITS_1NHALF)
	{
		uartx->CR2 |= ( 0x03<< USART_REG_CR2_STOP_BITS);//1.5 stop bits
	
	}else
	{
		uartx->CR2 |= ( 0x00<< USART_REG_CR2_STOP_BITS);// 1 stop bits
	}
	
}
/**
	* @brief  Configures the over sampling rate of the USART peripheral 
	* @param  *uartx : base address of the USART or UART peripheral
  * @param  over8     : if over8=1, then oversampling by 8 will be used , otherwise default Oversampling by 16
	* @retval None
	*/
void hal_uart_configure_over_sampling(USART_TypeDef *uartx, uint32_t over8)
{
	if(over8)
	{
		uartx->CR1 |= USART_REG_CR1_OVER8;
	}
	
}

/**
	* @brief  Program the given baudrate 
	* @param  *uartx : base address of the USART or UART peripheral
  * @param  baud     : baudrate value to be programmed
	* @retval None
	*/
void hal_uart_set_baud_rate(USART_TypeDef *uartx, uint32_t baud)
{
	uint32_t val;
	if (baud == USART_BAUD_9600)
	{
		val = 0x683;
	}
	else if (baud == USART_BAUD_115200)
	{
		val = 0x8A;
		
	}
	else
	{
		val = 0x8A;
	}
	uartx->BRR = val;
	
}

/**
	* @brief  Enable/Disable the TXE interrupt  
	* @param  *uartx : base address of the USART or UART peripheral
  * @param  txe_en     : if txe_en =1, then enable the TXE interrupt 
  * @retval None
	*/
void hal_uart_configure_txe_interrupt(USART_TypeDef *uartx, uint32_t txe_en)
{
	if(txe_en)
		uartx->CR1 |= USART_REG_CR1_TXE_INT_EN;
	else
		uartx->CR1 &= ~USART_REG_CR1_TXE_INT_EN;
}

/**
	* @brief  Enable/Disable the RXNE interrupt  
	* @param  *uartx : base address of the USART or UART peripheral
  * @param  rxne_en     : if rxne_en =1, then enable the RXNE interrupt 
  * @retval None
	*/
void hal_uart_configure_rxne_interrupt(USART_TypeDef *uartx, uint32_t rxne_en)
{
	if(rxne_en)
		uartx->CR1 |= USART_REG_CR1_RXNE_INT_EN;
	else
		uartx->CR1 &= ~USART_REG_CR1_RXNE_INT_EN;
}

/**
	* @brief  Enable/Disable the Error interrupt (Frame error, noise error, overrun error) 
	* @param  *uartx : base address of the USART or UART peripheral
  * @param  er_en     : if er_en =1, then enable the error interrupt 
  * @retval None
	*/
void hal_uart_configure_error_interrupt(USART_TypeDef *uartx, uint32_t er_en)
{
	if(er_en)
		uartx->CR3 |= USART_REG_CR3_ERR_INT_EN;
	else
		uartx->CR3 &= ~USART_REG_CR3_ERR_INT_EN;
	
}

/**
	* @brief  Enable/Disable the parity error interrupt  
	* @param  *uartx : base address of the USART or UART peripheral
  * @param  pe_en     : if pe_en =1, then enable the Parity Error interrupt 
  * @retval None
	*/
void hal_uart_configure_parity_error_interrupt(USART_TypeDef *uartx, uint32_t pe_en)
{
	if(pe_en)
		uartx->CR1 |= USART_REG_CR1_PEIE_INT_EN;
	else
		uartx->CR1 &= ~USART_REG_CR1_PEIE_INT_EN;
}


/**
  * @brief  UART error callbacks.
  * @param  huart: pointer to a uart_handle_t structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
  void hal_uart_error_cb(uart_handle_t *huart)
{
	while(1)
	{
		
		led_turn_on(GPIOD,LED_RED);
	}
}

/**
  * @brief  handle the TXE interrupt 
  * @param  huart: Pointer to a uart_handle_t structure that contains
  *                the configuration information for the specified UART module.
  * @retval none
  */
static void hal_uart_handle_TXE_interrupt(uart_handle_t *huart)
{
 
  uint32_t tmp1 = 0;
	uint8_t val;
  
  tmp1 = huart->tx_state;
  if(tmp1 == HAL_UART_STATE_BUSY_TX)
  { 
		val = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0x00FF);
		huart->Instance->DR = val;

		if(--huart->TxXferCount == 0)
		{
			/* Disable the UART TXE Interrupt */
			huart->Instance->CR1 &= ~USART_REG_CR1_TXE_INT_EN;

			/* Enable the UART Transmit Complete Interrupt */    
			huart->Instance->CR1 |= USART_REG_CR1_TCIE_INT_EN;
		}
	}
}
/**
  * @brief  Handle the RXNE interrupt 
  * @param  huart: pointer to a uart_handle_t structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
static void hal_uart_handle_RXNE_interrupt(uart_handle_t *huart)
{
 
  uint32_t tmp1 = 0;
  
  tmp1 = huart->rx_state; 
	
  if( tmp1 == HAL_UART_STATE_BUSY_RX )
  {
		//is application using parity ??
		if(huart->Init.Parity == UART_PARITY_NONE)
		{  //no parity 
			*huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
		}
		else
		{//yes, dont read the most significant bit, because its a parity bit 
			*huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x007F);
		}

		/* are we done with the reception ?? */
		if(--huart->RxXferCount == 0)
		{
			//yes, disable the RXNE interrupt 
			huart->Instance->CR1 &= ~USART_REG_CR1_RXNE_INT_EN;

			/* Disable the UART Parity Error Interrupt */
			huart->Instance->CR1 &= ~USART_REG_CR1_PEIE_INT_EN;

			/* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
			huart->Instance->CR3 &= ~USART_REG_CR3_ERR_INT_EN;
      
			//make the state ready for this handle 
			huart->rx_state = HAL_UART_STATE_READY;

			/*call the applicaton callback */
			if(huart->rx_cmp_cb)
					huart->rx_cmp_cb(&huart->RxXferSize);
		}
   }
}
  

/**
  * @brief Handle the Transmission Complete (TC) interrupt 
  * @param  huart: pointer to a uart_handle_t structure that contains
  *                the configuration information for the specified UART module.
  * @retval void
  */
static void  hal_uart_handle_TC_interrupt(uart_handle_t *huart)
{
	/* Disable the UART Transmit Complete Interrupt */    
	huart->Instance->CR1 &= ~USART_REG_CR1_TCIE_INT_EN;
	huart->tx_state = HAL_UART_STATE_READY;
	/*call the application callback */
	if(huart->tx_cmp_cb)
		huart->tx_cmp_cb(&huart->TxXferSize);
}

/**
	* @brief  Clear the error flag 
	* @param  *huart : pointer to a uart_handle_t structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
	*/
void hal_uart_clear_error_flag(uart_handle_t *huart)
{
	uint32_t tmpreg = 0x00;                
	tmpreg = huart->Instance->SR;        
	tmpreg = huart->Instance->DR;        
	
}


/******************************************************************************/
/*                                                                            */
/*                      Driver exposed APIs                                   */
/*                                                                            */
/******************************************************************************/


/**
	* @brief  API to do UART Peripheral initialization   
	* @param  *handle : pointer to the handle structure of the UART peripheral  
	* @retval None
	*/
void hal_uart_init(uart_handle_t *uart_handle)
{

	/* Configure the Word length */
	hal_uart_configure_word_length(uart_handle->Instance,uart_handle->Init.WordLength);
	
	/*Configure the number of STOP bits */
	hal_uart_configure_stop_bits(uart_handle->Instance,uart_handle->Init.StopBits);
	
	/*Configure the oversampling rate for the receive block */	
	hal_uart_configure_over_sampling(uart_handle->Instance,uart_handle->Init.OverSampling);
	
	/*Set the baud rate */
	hal_uart_set_baud_rate(uart_handle->Instance,uart_handle->Init.BaudRate);

	/*Enable the Transmit block of the UART peripheral */
	hal_uart_enable_disable_tx(uart_handle->Instance,uart_handle->Init.Mode);
	  
	/*Enable the Receive block of the UART Peripheral */
	hal_uart_enable_disable_rx(uart_handle->Instance,uart_handle->Init.Mode);
		
  /* Enable the UART peripheral */
	hal_uart_enable(uart_handle->Instance);
	
	uart_handle->tx_state  = HAL_UART_STATE_READY;
	uart_handle->rx_state  = HAL_UART_STATE_READY;
	uart_handle->ErrorCode = HAL_UART_ERROR_NONE;
		
}


/**
	* @brief  API to do UART data Transmission
	* @param  *uart_handle : pointer to the handle structure of the UART Peripheral 
  * @param  *buffer : holds the pointer to the TX buffer 
  * @param  len : len of the data to be TXed
	* @retval None
	*/
void hal_uart_tx(uart_handle_t *uart_handle, uint8_t *buffer, uint32_t len)
{
	/*Populate the application given inforamtions in to the UART handle structure */
	uart_handle->pTxBuffPtr = buffer;
	uart_handle->TxXferCount = len;
	uart_handle->TxXferSize = len;
	
	/* This handle is busy in doing the TX */
	uart_handle->tx_state = HAL_UART_STATE_BUSY_TX;
	
	/*Enable the UART Peripheral */
	hal_uart_enable(uart_handle->Instance);
	
	/*lets, enable the TXE interrupt */
	hal_uart_configure_txe_interrupt(uart_handle->Instance, 1);
}


/**
	* @brief  API to do UART data Reception  
	* @param  *handle : pointer to the handle structure of the UART peripheral  
  * @param  *buffer : holds the pointer to the RX buffer 
  * @param  len : len of the data to be RXed
	* @retval None
	*/
void hal_uart_rx(uart_handle_t *uart_handle, uint8_t *buffer, uint32_t len)
{
	uint32_t val;
/*Populate the application given inforamtions in to the UART handle structure */
	uart_handle->pRxBuffPtr = buffer;
	uart_handle->RxXferCount = len;
	uart_handle->RxXferSize = len;

	/* This handle is busy in doing the RX */
	uart_handle->rx_state = HAL_UART_STATE_BUSY_RX;

	/* Enable the UART Parity Error Interrupt */
	hal_uart_configure_parity_error_interrupt(uart_handle->Instance,1);


	/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
	hal_uart_configure_error_interrupt(uart_handle->Instance,1);

	val = uart_handle->Instance->DR;
	/* Enable the UART Data Register not empty Interrupt */
	hal_uart_configure_rxne_interrupt(uart_handle->Instance,1);
	
}


/**
  * @brief  This function handles UART interrupt request.
  * @param  huart: pointer to a uart_handle_t structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void hal_uart_handle_interrupt(uart_handle_t *huart)
{
    uint32_t tmp1 = 0, tmp2 = 0;

	tmp1 = huart->Instance->SR & USART_REG_SR_PE_FLAG;
  tmp2 = huart->Instance->CR1 & USART_REG_CR1_PEIE_INT_EN;
  /* UART parity error interrupt occurred ------------------------------------*/
  if((tmp1) && (tmp2))
  { 
		hal_uart_clear_error_flag(huart);
    
    huart->ErrorCode |= HAL_UART_ERROR_PE;
  }
  
	tmp1 = huart->Instance->SR & USART_REG_SR_FE_FLAG;
	tmp2 = huart->Instance->CR3 & USART_REG_CR3_ERR_INT_EN;
  /* UART frame error interrupt occurred -------------------------------------*/
  if((tmp1 ) && (tmp2 ))
  { 
    hal_uart_clear_error_flag(huart);
    
    huart->ErrorCode |= HAL_UART_ERROR_FE;
  }
  
  
	tmp1 = huart->Instance->SR & USART_REG_SR_NE_FLAG;
  tmp2 = huart->Instance->CR3 & USART_REG_CR3_ERR_INT_EN;
  /* UART noise error interrupt occurred -------------------------------------*/
  if((tmp1 ) && (tmp2 ))
  { 
     hal_uart_clear_error_flag(huart);
    
    huart->ErrorCode |= HAL_UART_ERROR_NE;
  }
  
 
	tmp1 = huart->Instance->SR & USART_REG_SR_ORE_FLAG;
  tmp2 = huart->Instance->CR3 & USART_REG_CR3_ERR_INT_EN;
  /* UART Over-Run interrupt occurred ----------------------------------------*/
  if((tmp1) && (tmp2))
  { 
     hal_uart_clear_error_flag(huart);
    
    huart->ErrorCode |= HAL_UART_ERROR_ORE;
  }
  
 
	 tmp1 = huart->Instance->SR & USART_REG_SR_RXNE_FLAG;
   tmp2 = huart->Instance->CR1 & USART_REG_CR1_RXNE_INT_EN;
  /* UART in mode Receiver ---------------------------------------------------*/
  if((tmp1 ) && (tmp2))
  { 
    hal_uart_handle_RXNE_interrupt(huart);
  }
  
  
	tmp1 = huart->Instance->SR & USART_REG_SR_TXE_FLAG;
  tmp2 = huart->Instance->CR1 & USART_REG_CR1_TXE_INT_EN;
  /* UART in mode Transmitter ------------------------------------------------*/
  if((tmp1 ) && (tmp2))
  {
    hal_uart_handle_TXE_interrupt(huart);
  }
  
	tmp1 = huart->Instance->SR & USART_REG_SR_TC_FLAG;
	tmp2 = huart->Instance->CR1 & USART_REG_CR1_TCIE_INT_EN;
  /* UART in mode Transmitter end --------------------------------------------*/
  if((tmp1 ) && (tmp2))
  {
    hal_uart_handle_TC_interrupt(huart);
  }

  if(huart->ErrorCode != HAL_UART_ERROR_NONE)
  {
    /* Set the UART state ready to be able to start again the process */
    huart->tx_state = HAL_UART_STATE_READY;
		huart->rx_state = HAL_UART_STATE_READY;
    
    hal_uart_error_cb(huart);
  }  
}


