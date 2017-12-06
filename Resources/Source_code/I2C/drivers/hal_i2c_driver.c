
#include "hal_i2c_driver.h"
#include "led.h"

void hal_i2c_enable_peripheral(I2C_TypeDef *i2cx)
{
	i2cx->CR1 |= I2C_REG_CR1_ENABLE_I2C;
}

void hal_i2c_disable_peripheral(I2C_TypeDef *i2cx)
{
	i2cx->CR1 &= ~I2C_REG_CR1_ENABLE_I2C;
	
}



void hal_i2c_set_own_address1(I2C_TypeDef *i2cx, uint32_t own_address)
{
	i2cx->OAR1 &= ~( 0x7f << 1);
	i2cx->OAR1 |=  (own_address << 1);
}


void hal_i2c_manage_clock_stretch(I2C_TypeDef *i2cx, uint32_t no_stretch)
{
	if(no_stretch)
	{
		i2cx->CR1 |= I2C_REG_CR1_NOSTRETCH;
	}else
	{
		i2cx->CR1 &= ~I2C_REG_CR1_NOSTRETCH;
	}
}


void hal_i2c_configure_ccr(I2C_TypeDef *i2cx, uint32_t pclk, uint32_t clkspeed, uint32_t duty_cycle)
{
	
	
	double Thigh, Tlow,  Tpclk;
	uint32_t ccr;
	if(clkspeed <= 100000)
	{
		/* configure ccr for standard mode */
		ccr = ( pclk * 1000000) / (clkspeed << 1);
		
	}else
	{
		 if(duty_cycle == I2C_FM_DUTY_2)
		 {
			 ccr = ( pclk * 1000000)/(3 * clkspeed);
			 
		 }
		 
		  if(duty_cycle == I2C_FM_DUTY_16BY9)
		 {
			 /* this is to reach 400khz in fm mode */
			ccr = ( pclk * 1000000)/(25 * clkspeed);
			 
		 }	
	}
	
	i2cx->CCR |= ccr;

}
	

void hal_i2c_rise_time_configuration(I2C_TypeDef *i2cx,uint32_t freqrange, uint32_t ClockSpeed)
{
	  /*---------------------------- I2Cx TRISE Configuration --------------------*/
  /* Configure I2Cx: Rise Time */
//  hi2c->Instance->TRISE = I2C_RISE_TIME(freqrange, hi2c->Init.ClockSpeed);
	
	uint32_t trise;
	if( ClockSpeed <= 100000)
	{
		trise = freqrange +1;
	}else
	{
		trise = (((freqrange * 300) / 1000) + 1);
	}

		i2cx->TRISE  &= ~(0x3F);
		i2cx->TRISE  |= trise;
	
}

void hal_i2c_clk_init(I2C_TypeDef *i2cx, uint32_t clkspeed, uint32_t duty_cycle)
{
	uint32_t pclk = I2C_PERIPHERAL_CLK_FREQ_8MHZ;
	i2cx->CR2 |= (pclk );
	hal_i2c_rise_time_configuration(i2cx,pclk, clkspeed);
	hal_i2c_configure_ccr(i2cx,pclk,clkspeed,duty_cycle);
}


void hal_i2c_set_addressing_mode(I2C_TypeDef *i2cx, uint32_t adr_mode)
{
	if(adr_mode == I2C_ADDRMODE_10BI)
		i2cx->OAR1 |= I2C_REG_OAR1_ADDRMODE;
	else 
		i2cx->OAR1 &= ~I2C_REG_OAR1_ADDRMODE;
}



void hal_i2c_set_fm_mode_duty_cycle(I2C_TypeDef *i2cx, uint32_t duty_cycle)
{
	if(duty_cycle == I2C_FM_DUTY_16BY9 )
	{
     i2cx->CCR |= I2C_REG_CCR_DUTY;
	}else
  {
     i2cx->CCR &= ~I2C_REG_CCR_DUTY;
	}		
	
}


void hal_i2c_manage_ack(I2C_TypeDef *i2cx, uint32_t ack_noack)
{
	if(ack_noack == I2C_ACK_ENABLE)
		i2cx->CR1 |= I2C_REG_CR1_ACK;
	else
		i2cx->CR1 &= ~I2C_REG_CR1_ACK;
}

void hal_i2c_generate_start_condition(I2C_TypeDef *i2cx)
{
	i2cx->CR1 |= I2C_REG_CR1_START_GEN;
	
}

void hal_i2c_generate_stop_condition(I2C_TypeDef *i2cx)
{
	i2cx->CR1 |= I2C_REG_CR1_STOP_GEN;
	
}

void hal_i2c_configure_tx_rx_interrupt(I2C_TypeDef *i2cx, uint32_t val)
{
	if(val)
	  i2cx->CR2 |= I2C_REG_CR2_BUF_INT_ENABLE;
	else
		i2cx->CR2 &= ~I2C_REG_CR2_BUF_INT_ENABLE;

}




void hal_i2c_configure_error_interrupt(I2C_TypeDef *i2cx, uint32_t val)
{
	
	if(val)
	  i2cx->CR2 |= I2C_REG_CR2_ERR_INT_ENABLE;
	else
		i2cx->CR2 &= ~I2C_REG_CR2_ERR_INT_ENABLE;
	
}

void 	hal_i2c_configure_evt_interrupt(I2C_TypeDef *i2cx, uint32_t val)
{
		if(val)
			i2cx->CR2 |= I2C_REG_CR2_EVT_INT_ENABLE;
	  else
		  i2cx->CR2 &= ~I2C_REG_CR2_EVT_INT_ENABLE;
	
}

uint8_t is_bus_busy(I2C_TypeDef *i2cx)
{
	if(i2cx->SR2 & I2C_REG_SR2_BUS_BUSY_FLAG )
		return 1;
	else
		return 0;
}

uint8_t i2c_wait_untill_sb_set(I2C_TypeDef *i2cx)
{
	//EV5: SB=1, cleared by reading SR1 register followed by writing DR register with Address.
	
	if (i2cx->SR1 & I2C_REG_SR1_SB_FLAG )
	{
		return 1 ;
	}
	return 0;
}


uint8_t i2c_wait_untill_addr_set(I2C_TypeDef *i2cx)
{
	//EV6: ADDR=1, cleared by reading SR1 register followed by reading SR2.
	
	if (i2cx->SR1 & I2C_REG_SR1_ADDR_SENT_FLAG )
	{
		return 1 ;
	}
	return 0;
}


void hal_i2c_init(i2c_handle_t *handle)
{
	hal_i2c_clk_init(handle->Instance, handle->Init.ClockSpeed,handle->Init.DutyCycle);
	hal_i2c_set_addressing_mode(handle->Instance, handle->Init.AddressingMode);
	hal_i2c_manage_ack(handle->Instance, handle->Init.ack_enable);
	hal_i2c_manage_clock_stretch(handle->Instance,handle->Init.NoStretchMode);
	hal_i2c_set_own_address1(handle->Instance,handle->Init.OwnAddress1);
	
}

void hal_i2c_send_addr_first(I2C_TypeDef *i2cx, uint8_t address)
{
	
	i2cx->DR = address;
	
}



void clear_addr_flag(I2C_TypeDef *i2cx)
{
	uint16_t val;
	
	val = i2cx->SR1;
	val = i2cx->SR2;
	
}


void hal_i2c_master_tx(i2c_handle_t *handle, uint8_t slave_address, uint8_t *buffer, uint32_t len)
{

	hal_i2c_enable_peripheral(handle->Instance);
	
	/* doesnt care for PE = 0 */
	while(is_bus_busy(handle->Instance) ); //need to include timeout 
	
	
	
	 /* Disable Pos */
    handle->Instance->CR1 &= ~I2C_CR1_POS;

	handle->State = HAL_I2C_STATE_BUSY_TX;
	
	handle->pBuffPtr = buffer;
	handle->XferCount = len;
	handle->XferSize = len;
	
	
		
  hal_i2c_generate_start_condition(handle->Instance);
	
	/* wait till sb is set */
	
	while(! i2c_wait_untill_sb_set(handle->Instance) );
	
	//clear_sb_flag(); ???
	
	hal_i2c_send_addr_first(handle->Instance,slave_address);
	
	
	while ( ! i2c_wait_untill_addr_set(handle->Instance) );
	
	clear_addr_flag(handle->Instance); // IS THIS really needed ??
	
	/* enable the buff, err , event interrupts */
	hal_i2c_configure_tx_rx_interrupt(handle->Instance,1);
	hal_i2c_configure_error_interrupt(handle->Instance,1);
	hal_i2c_configure_evt_interrupt(handle->Instance,1);
	
	
}
void hal_i2c_master_rx(i2c_handle_t *handle, uint8_t slave_addr, uint8_t *buffer, uint32_t len)
{


	
	hal_i2c_enable_peripheral(handle->Instance);
	
	while(is_bus_busy(handle->Instance) );
	
	handle->Instance->CR1 &= ~I2C_CR1_POS;
	
	handle->State = HAL_I2C_STATE_BUSY_RX;
	
	handle->pBuffPtr = buffer;
	handle->XferCount = len;
	handle->XferSize = len;
	
	handle->Instance->CR1 |= I2C_CR1_ACK;
	
	hal_i2c_generate_start_condition(handle->Instance);
	
	/* wait till sb is set */
	
	
	while(! i2c_wait_untill_sb_set(handle->Instance) );
	
	//clear_sb_flag(); ???
	
	hal_i2c_send_addr_first(handle->Instance,slave_addr);
	
	while ( ! i2c_wait_untill_addr_set(handle->Instance) );
	
	clear_addr_flag(handle->Instance); // IS THIS really needed ??
	

	/* Enable the buff, err , event interrupts */
	hal_i2c_configure_tx_rx_interrupt(handle->Instance,1);
	hal_i2c_configure_error_interrupt(handle->Instance,1);
	hal_i2c_configure_evt_interrupt(handle->Instance,1);
}

void hal_i2c_slave_tx(i2c_handle_t *handle, uint8_t *buffer, uint32_t len)
{
	hal_i2c_enable_peripheral(handle->Instance);
	
	//while(is_bus_busy(handle->Instance) );
	
	handle->Instance->CR1 &= ~I2C_CR1_POS;
	
	handle->State = HAL_I2C_STATE_BUSY_TX;
	
	handle->pBuffPtr = buffer;
	handle->XferCount = len;
	handle->XferSize = len;
	
	 /* Enable Address Acknowledge */
	handle->Instance->CR1 |= I2C_CR1_ACK;
	
			/* ENABLE the buff, err , event interrupts */
	hal_i2c_configure_tx_rx_interrupt(handle->Instance,1);
	hal_i2c_configure_error_interrupt(handle->Instance,1);
	hal_i2c_configure_evt_interrupt(handle->Instance,1);
	
}
void hal_i2c_slave_rx(i2c_handle_t *handle, uint8_t *buffer, uint32_t len)
{
	uint32_t val;
	
	hal_i2c_enable_peripheral(handle->Instance);
	
	//while(is_bus_busy(handle->Instance) );
	
	handle->Instance->CR1 &= ~I2C_CR1_POS;
	handle->State = HAL_I2C_STATE_BUSY_RX;
	
	handle->pBuffPtr = buffer;
	handle->XferCount = len;
	handle->XferSize = len;
	
	
	
	handle->Instance->CR1 |= I2C_CR1_ACK;
	
		/* disable the buff, err , event interrupts */
	hal_i2c_configure_tx_rx_interrupt(handle->Instance,1);
	hal_i2c_configure_error_interrupt(handle->Instance,1);
	hal_i2c_configure_evt_interrupt(handle->Instance,1);

#if 0
	val = handle->Instance->CR2;
	val = handle->Instance->CR1;
		val = handle->Instance->OAR1;
#endif 
}


