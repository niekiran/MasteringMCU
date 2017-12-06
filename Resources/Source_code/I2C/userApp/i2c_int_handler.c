#include<stdint.h>
//#include "stm32f4xx_hal.h"
#include "hal_i2c_driver.h"
#include "led.h"
#include "stm32f4xx_hal_i2c.h"

/**
  * @brief  Handle TXE flag for Master
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
static void I2C_MasterTransmit_TXE(i2c_handle_t *hi2c)
{
  /* Write data to DR */
  hi2c->Instance->DR = (*hi2c->pBuffPtr++);
  hi2c->XferCount--;

  if(hi2c->XferCount == 0)
  {
    /* Disable BUF interrupt */
		hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
  }

}
/**
  * @brief  Memory Tx Transfer completed callbacks.
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
 void HAL_I2C_MemTxCpltCallback(i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_I2C_TxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Master Tx Transfer completed callbacks.
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
  void HAL_I2C_MasterTxCpltCallback(i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_I2C_TxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Memory Rx Transfer completed callbacks.
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
 void HAL_I2C_MemRxCpltCallback(i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_I2C_TxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Slave Rx Transfer completed callbacks.
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
 void HAL_I2C_SlaveRxCpltCallback(i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_I2C_TxCpltCallback could be implemented in the user file
   */
}

/** @brief  Slave Tx Transfer completed callbacks.
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
  void HAL_I2C_SlaveTxCpltCallback(i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_I2C_TxCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Master Rx Transfer completed callbacks.
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
 void HAL_I2C_MasterRxCpltCallback(i2c_handle_t *hi2c)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hi2c);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_I2C_TxCpltCallback could be implemented in the user file */
}
  

/**
  * @brief  Handle BTF flag for Master transmitter
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
static void I2C_MasterTransmit_BTF(i2c_handle_t *hi2c)
{
  if(hi2c->XferCount != 0)
  {
    /* Write data to DR */
    hi2c->Instance->DR = (*hi2c->pBuffPtr++);
    hi2c->XferCount--;
  }
  else
  {
    /* Disable EVT, BUF and ERR interrupt */
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;

    /* Generate Stop */
    hi2c->Instance->CR1 |= I2C_REG_CR1_STOP_GEN;

    if(hi2c->State == HAL_I2C_STATE_MEM_BUSY_TX)
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MemTxCpltCallback(hi2c);
    }
    else
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MasterTxCpltCallback(hi2c);
    }
  }

}



/**
  * @brief  Handle BTF flag for Master receiver
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
static void I2C_MasterReceive_BTF(i2c_handle_t *hi2c)
{
  if(hi2c->XferCount == 3)
  {
    /* Disable Acknowledge */
    hi2c->Instance->CR1 &= ~I2C_REG_CR1_ACK;

    /* Read data from DR */
    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;
  }
  else if(hi2c->XferCount == 2)
  {
    /* Generate Stop */
    hi2c->Instance->CR1 |= I2C_REG_CR1_STOP_GEN;

    /* Read data from DR */
    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;

    /* Read data from DR */
    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;

    /* Disable EVT and ERR interrupt */
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;

    if(hi2c->State == HAL_I2C_STATE_MEM_BUSY_RX)
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MemRxCpltCallback(hi2c);
    }
    else
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MasterRxCpltCallback(hi2c);
    }
  }
  else
  {
    /* Read data from DR */
    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;
  }
 
}

/**
  * @brief  Handle ADD flag for Slave
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
static void I2C_Slave_ADDR(i2c_handle_t *hi2c)
{
	uint32_t tmpreg;
  /* Clear ADDR flag */
    tmpreg = hi2c->Instance->SR1;  //read SR1     
    tmpreg = hi2c->Instance->SR2;  //read SR2
}

void hal_clear_stop_flag(i2c_handle_t *hi2c)
{
	 uint32_t tmpreg;
	 tmpreg = hi2c->Instance->SR1;      //reading from SR1
   hi2c->Instance->CR1 |= I2C_REG_CR1_ENABLE_I2C;  //writing to SR1
}

/**
  * @brief  Handle STOPF flag for Slave
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
static void I2C_Slave_STOPF(i2c_handle_t *hi2c)
{
	
  /* Disable EVT, BUF and ERR interrupt */
		hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
		hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
		hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;

  /* Clear STOPF flag */
  hal_clear_stop_flag(hi2c);
	
	 

  /* Disable Acknowledge */
  hi2c->Instance->CR1 &= ~I2C_CR1_ACK;

  hi2c->State = HAL_I2C_STATE_READY;

  HAL_I2C_SlaveRxCpltCallback(hi2c);

  
}

/**
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
static void I2C_Slave_AF(i2c_handle_t *hi2c)
{
  /* Disable EVT, BUF and ERR interrupt */
	hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
	hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
	hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;

  /* Clear AF flag */
  hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_AF_FAILURE_FLAG);

  /* Disable Acknowledge */
  hi2c->Instance->CR1 &= ~I2C_REG_CR1_ACK;

  hi2c->State = HAL_I2C_STATE_READY;

  HAL_I2C_SlaveTxCpltCallback(hi2c);

 
}


/**
  * @brief  Handle TXE flag for Slave
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
static void I2C_SlaveTransmit_TXE(i2c_handle_t *hi2c)
{
  if(hi2c->XferCount != 0)
  {
    /* Write data to DR */
    hi2c->Instance->DR = (*hi2c->pBuffPtr++);
    hi2c->XferCount--;
  }
  
}

/**
  * @brief  Handle BTF flag for Slave transmitter
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
static void I2C_SlaveTransmit_BTF(i2c_handle_t *hi2c)
{
  if(hi2c->XferCount != 0)
  {
    /* Write data to DR */
    hi2c->Instance->DR = (*hi2c->pBuffPtr++);
    hi2c->XferCount--;
  }
  
}


/**
  * @brief  Handle RXNE flag for Master
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
static void I2C_MasterReceive_RXNE(i2c_handle_t *hi2c)
{
  uint32_t tmp = 0;

  tmp = hi2c->XferCount;
  if(tmp > 3)
  {
    /* Read data from DR */
    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;
  }
  else if((tmp == 2) || (tmp == 3))
  {
    /* Disable BUF interrupt */
		hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
  }
  else
  {
    /* Disable EVT, BUF and ERR interrupt */
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_EVT_INT_ENABLE;
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_BUF_INT_ENABLE;
			hi2c->Instance->CR2 &= ~ I2C_REG_CR2_ERR_INT_ENABLE;

    /* Read data from DR */
    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;

    if(hi2c->State == HAL_I2C_STATE_MEM_BUSY_RX)
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MemRxCpltCallback(hi2c);
    }
    else
    {
      hi2c->State = HAL_I2C_STATE_READY;

      HAL_I2C_MasterRxCpltCallback(hi2c);
    }
  }
  
}

/**
  * @brief  Handle BTF flag for Slave receiver
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
static void I2C_SlaveReceive_BTF(i2c_handle_t *hi2c)
{
  if(hi2c->XferCount != 0)
  {
    /* Read data from DR */
    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;
  }
  
}

/**
  * @brief  Handle RXNE flag for Slave
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
static void I2C_SlaveReceive_RXNE(i2c_handle_t *hi2c)
{
  if(hi2c->XferCount != 0)
  {
    /* Read data from DR */
    (*hi2c->pBuffPtr++) = hi2c->Instance->DR;
    hi2c->XferCount--;
  }
  
}


/**
  * @brief  This function handles I2C event interrupt request.
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
void HAL_I2C_EV_IRQHandler(i2c_handle_t *hi2c)
{
  uint32_t tmp1 = 0, tmp2 = 0, tmp3 = 0, tmp4 = 0;
  /* Master mode selected */
  if(( hi2c->Instance->SR2 & I2C_REG_SR2_MSL_FLAG ))
  {
    /* I2C in mode Transmitter -----------------------------------------------*/
    if(( hi2c->Instance->SR2 & I2C_REG_SR2_TRA_FLAG))
    {
     tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_TXE_FLAG );
			tmp2 =  (hi2c->Instance->CR2 & I2C_REG_CR2_BUF_INT_ENABLE );
			tmp3 = (hi2c->Instance->SR1 & I2C_REG_SR1_BTF_FLAG );
			tmp4 = (hi2c->Instance->CR2 & I2C_REG_CR2_EVT_INT_ENABLE );
      /* TXE set and BTF reset -----------------------------------------------*/
      if(tmp1 && tmp2 && (! tmp3))
      {
        I2C_MasterTransmit_TXE(hi2c);
      }
      /* BTF set -------------------------------------------------------------*/
      else if((tmp3 && tmp4 ))
      {
        I2C_MasterTransmit_BTF(hi2c);
      }
    }
    /* I2C in mode Receiver --------------------------------------------------*/
    else
    {
      
			tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_RXNE_FLAG );
			tmp2 =  (hi2c->Instance->CR2 & I2C_REG_CR2_BUF_INT_ENABLE );
			tmp3 = (hi2c->Instance->SR1 & I2C_REG_SR1_BTF_FLAG );
			tmp4 = (hi2c->Instance->CR2 & I2C_REG_CR2_EVT_INT_ENABLE );
      /* RXNE set and BTF reset -----------------------------------------------*/
      if((tmp1) && (tmp2) && (! tmp3))
      {
        I2C_MasterReceive_RXNE(hi2c);
      }
      /* BTF set -------------------------------------------------------------*/
      else if((tmp3) && (tmp4))
      {
        I2C_MasterReceive_BTF(hi2c);
      }
    }
  }
  /* Slave mode selected */
  else
  {
    
		tmp1 = ( hi2c->Instance->SR1 & I2C_REG_SR1_ADDR_FLAG);
    tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_EVT_INT_ENABLE );
		tmp3 = ( hi2c->Instance->SR1 & I2C_REG_SR1_STOP_DETECTION_FLAG);
    tmp4 = ( hi2c->Instance->SR2 & I2C_REG_SR2_TRA_FLAG);
    /* ADDR set --------------------------------------------------------------*/
    if((tmp1 ) && (tmp2))
    {
			led_turn_on(GPIOD,LED_GREEN);
      I2C_Slave_ADDR(hi2c);
    }
    /* STOPF set --------------------------------------------------------------*/
    else if((tmp3) && (tmp2))
    {
      I2C_Slave_STOPF(hi2c);
    }
    /* I2C in mode Transmitter -----------------------------------------------*/
    else if(tmp4)
    {
			tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_TXE_FLAG );
			tmp2 =  (hi2c->Instance->CR2 & I2C_REG_CR2_BUF_INT_ENABLE );
			tmp3 = (hi2c->Instance->SR1 & I2C_REG_SR1_BTF_FLAG );
			tmp4 = (hi2c->Instance->CR2 & I2C_REG_CR2_EVT_INT_ENABLE );
      /* TXE set and BTF reset -----------------------------------------------*/
      if((tmp1) && (tmp2) && (! tmp3))
      {
        I2C_SlaveTransmit_TXE(hi2c);
      }
      /* BTF set -------------------------------------------------------------*/
      else if((tmp3) && (tmp4))
      {
        I2C_SlaveTransmit_BTF(hi2c);
      }
    }
    /* I2C in mode Receiver --------------------------------------------------*/
    else
    {
      tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_RXNE_FLAG );
			tmp2 =  (hi2c->Instance->CR2 & I2C_REG_CR2_BUF_INT_ENABLE );
			tmp3 = (hi2c->Instance->SR1 & I2C_REG_SR1_BTF_FLAG );
			tmp4 = (hi2c->Instance->CR2 & I2C_REG_CR2_EVT_INT_ENABLE );
      /* RXNE set and BTF reset ----------------------------------------------*/
      if((tmp1 ) && (tmp2 ) && (! tmp3))
      {
        I2C_SlaveReceive_RXNE(hi2c);
      }
      /* BTF set -------------------------------------------------------------*/
      else if((tmp3) && (tmp4))
      {
        I2C_SlaveReceive_BTF(hi2c);
      }
    }
  }
}

/**
  * @brief  I2C error callbacks
  * @param  I2cHandle: I2C handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_I2C_ErrorCallback(i2c_handle_t *I2cHandle)
{
  while(1)
  {
		led_toggle(GPIOD,LED_RED);
  }
}

/**
  * @brief  This function handles I2C error interrupt request.
  * @param  hi2c: pointer to a i2c_handle_t structure that contains
  *         the configuration information for I2C module
  * @retval HAL status
  */
void HAL_I2C_ER_IRQHandler(i2c_handle_t *hi2c)
{
  uint32_t tmp1 = 0, tmp2 = 0, tmp3 = 0;
	tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_BUS_ERROR_FLAG);
  tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_ERR_INT_ENABLE );
  /* I2C Bus error interrupt occurred ----------------------------------------*/
  if((tmp1) && (tmp2 ))
  {
    hi2c->ErrorCode |= HAL_I2C_ERROR_BERR;

    /* Clear BERR flag */
		hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_BUS_ERROR_FLAG);
  }

  tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_ARLO_FLAG);
  tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_ERR_INT_ENABLE );
  /* I2C Arbitration Loss error interrupt occurred ---------------------------*/
  if((tmp1 ) && (tmp2))
  {
    hi2c->ErrorCode |= HAL_I2C_ERROR_ARLO;

    /* Clear ARLO flag */
    hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_ARLO_FLAG);
  }

  tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_AF_FAILURE_FLAG);
  tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_ERR_INT_ENABLE );
  /* I2C Acknowledge failure error interrupt occurred ------------------------*/
  if((tmp1 ) && (tmp2 ))
  {
    tmp1 = ( hi2c->Instance->SR2 & I2C_REG_SR2_MSL_FLAG );
    tmp2 = hi2c->XferCount;
    tmp3 = hi2c->State;
    if(( ! tmp1 ) && (tmp2 == 0) && (tmp3 == HAL_I2C_STATE_BUSY_TX))
    {
      I2C_Slave_AF(hi2c);
    }
    else
    {
      hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
      /* Clear AF flag */
       hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_AF_FAILURE_FLAG);
    }
  }

 tmp1 =  (hi2c->Instance->SR1 & I2C_REG_SR1_OVR_FLAG);
  tmp2 = (hi2c->Instance->CR2 & I2C_REG_CR2_ERR_INT_ENABLE );
  /* I2C Over-Run/Under-Run interrupt occurred -------------------------------*/
  if((tmp1) && (tmp2))
  {
    hi2c->ErrorCode |= HAL_I2C_ERROR_OVR;
    /* Clear OVR flag */
    hi2c->Instance->SR1 &= ~ (I2C_REG_SR1_OVR_FLAG);
  }

  if(hi2c->ErrorCode != HAL_I2C_ERROR_NONE)
  {
    hi2c->State = HAL_I2C_STATE_READY;
    
    /* Disable Pos bit in I2C CR1 when error occurred in Master/Mem Receive IT Process */
    hi2c->Instance->CR1 &= ~I2C_REG_CR1_POS;
    
    HAL_I2C_ErrorCallback(hi2c);
  }
}
