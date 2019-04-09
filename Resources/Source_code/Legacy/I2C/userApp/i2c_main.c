#include <stdint.h>

#include "hal_i2c_driver.h"
#include "hal_gpio_driver.h"
#include "i2c_main.h"
#include "led.h"
//#define I2C_MASTER_MODE_EN

#define SLAVE_OWN_ADDRESS      (uint8_t)0x53;
#define SLAVE_ADDRESS_READ    (uint8_t) 0xA7
#define SLAVE_ADDRESS_WRITE    (uint8_t) 0xA6

#define GENERAL_CALL_ADDRESS    (uint8_t)0x00

#define MASTER_WRITE_CMD       0xC1
#define MASTER_READ_CMD        0XC2

#define READ_LEN    5
#define WRITE_LEN   5


uint8_t master_tx_buffer[5]={0xa5, 0x55, 0xa5, 0x55, 0xb0};
uint8_t master_rx_buffer[5];

uint8_t slave_tx_buffer[5]={0xa5, 0x55, 0xa5, 0x55, 0xc0};
uint8_t slave_rx_buffer[5];

uint8_t master_write_req;
uint8_t master_read_req;

uint8_t slave_rcv_cmd;

extern void  hal_i2c_enable_peripheral(I2C_TypeDef *i2cx);
extern  void hal_gpio_driver_set_alternate_function(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint16_t alt_fun_value);
extern void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
extern void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
extern void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);

void delay_gen( )
{
	uint32_t cnt = 500000;
	while(cnt--);
}

i2c_handle_t i2c_handle;
int TestReady = 0;
void i2c_gpio_init()
{
		gpio_pin_conf_t i2c_scl, i2c_sda;
	
	 _HAL_RCC_GPIOB_CLK_ENABLE();

	
	i2c_scl.pin = I2C1_SCL_LINE;
	i2c_scl.mode = GPIO_PIN_ALT_FUN_MODE;
	i2c_scl.op_type = GPIO_PIN_OP_TYPE_OPEN_DRAIN;
	i2c_scl.pull = GPIO_PIN_PULL_UP;
	i2c_scl.speed = GPIO_PIN_SPEED_HIGH;
	
	hal_gpio_set_alt_function(GPIOB,I2C1_SCL_LINE,GPIO_PIN_AF4_I2C123);
	
	hal_gpio_init(GPIOB, &i2c_scl);
	
 
	
	
	i2c_sda.pin = I2C1_SDA_LINE;
	i2c_sda.mode = GPIO_PIN_ALT_FUN_MODE;
	i2c_sda.op_type = GPIO_PIN_OP_TYPE_OPEN_DRAIN;
	i2c_sda.pull = GPIO_PIN_PULL_UP;
	i2c_sda.speed = GPIO_PIN_SPEED_HIGH;
	
	hal_gpio_set_alt_function(GPIOB,I2C1_SDA_LINE,GPIO_PIN_AF4_I2C123);
	hal_gpio_init(GPIOB, &i2c_sda);

	
}

/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if ((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
}

void gpio_btn_interrupt_handler(void)
{
	hal_gpio_clear_interrupt(0);

	TestReady = SET;
	
}

int main(void)
{
	uint32_t val;
	led_init();
 
	i2c_gpio_init();
	
		/* Configure USER Button */
#ifdef I2C_MASTER_MODE_EN
 hal_gpio_configure_interrupt(0, INT_FALLING_EDGE, gpio_btn_interrupt_handler);

#endif
	
	_HAL_RCC_I2C1_CLK_ENABLE() ;
	i2c_handle.Instance = I2C_1;
	i2c_handle.Init.ack_enable = I2C_ACK_ENABLE;
	i2c_handle.Init.AddressingMode = I2C_ADDRMODE_7BIT; 
	i2c_handle.Init.ClockSpeed = 100000;
	i2c_handle.Init.DutyCycle = I2C_FM_DUTY_2; //care needs to taken 
	i2c_handle.Init.GeneralCallMode = 0;
	i2c_handle.Init.NoStretchMode = I2C_ENABLE_CLK_STRETCH;
	i2c_handle.Init.OwnAddress1 = SLAVE_OWN_ADDRESS;

	
	
  NVIC_EnableIRQ(I2Cx_ER_IRQn);
  NVIC_EnableIRQ(I2Cx_EV_IRQn);
	
	hal_i2c_init(&i2c_handle);
  hal_i2c_enable_peripheral(i2c_handle.Instance);
	
		hal_gpio_enable_interrupt(0);
	
	//val = i2c_handle.Instance->CR1;
	i2c_handle.State = HAL_I2C_STATE_READY;
	
	
#ifdef I2C_MASTER_MODE_EN

	 /* Wait for user Button press before starting the communication. Toggles LED3 until then */
  while(TestReady != SET)
  {
    led_toggle(GPIOD,LED_ORANGE);
		//LED3 (orange)
    delay_gen();
  }
	
	led_turn_off(GPIOD,LED_ORANGE);

#endif 	
	
while(1)
{
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	
#ifdef I2C_MASTER_MODE_EN
	/* first send the master write cmd to slave */
	master_write_req = MASTER_WRITE_CMD;
	hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_write_req,1);
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	
	master_write_req = WRITE_LEN;
	/* Now send the number of bytes to be written */
	hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_write_req,1);
	
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	
	/* NOW send the data stream */
	hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,master_tx_buffer,WRITE_LEN);
	
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	

	
	/* first send the master read cmd to slave */
	master_read_req = MASTER_READ_CMD;
	hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_read_req,1);
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	
	master_read_req = READ_LEN;
	/* Now send the number of bytes to be read */
	hal_i2c_master_tx(&i2c_handle,SLAVE_ADDRESS_WRITE,(uint8_t*)&master_read_req,1);
	
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	
	memset(master_rx_buffer,0, 5);
	/* NOW read the data stream */
	hal_i2c_master_rx(&i2c_handle,SLAVE_ADDRESS_READ,master_rx_buffer,READ_LEN);
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	
	if ( Buffercmp(slave_tx_buffer,master_rx_buffer,READ_LEN))
	{
		led_turn_on(GPIOD,LED_RED);
	}else
		led_toggle(GPIOD,LED_BLUE);
	
	//led_turn_on(GPIOD,LED_ORANGE);
	
	delay_gen();
	
#else
	/* first rcv the commmand from the master */
	hal_i2c_slave_rx(&i2c_handle,&slave_rcv_cmd,1);
	while(i2c_handle.State != HAL_I2C_STATE_READY);
	
	if(slave_rcv_cmd == MASTER_WRITE_CMD)
	{
		//prepare to rcv from the master 
		//first rcv no bytes to be written by master 
		hal_i2c_slave_rx(&i2c_handle,&slave_rcv_cmd,1);
  	while(i2c_handle.State != HAL_I2C_STATE_READY);
		memset(slave_rx_buffer,0, sizeof(slave_rx_buffer));
		hal_i2c_slave_rx(&i2c_handle,slave_rx_buffer,slave_rcv_cmd);
		while(i2c_handle.State != HAL_I2C_STATE_READY);
		
		if ( Buffercmp(slave_rx_buffer,master_tx_buffer,READ_LEN))
		{
			led_turn_on(GPIOD,LED_RED);
		}else
			led_toggle(GPIOD,LED_BLUE);
	}
	
	if(slave_rcv_cmd == MASTER_READ_CMD)
	{
		//prepare to send data to the  master 
		//first rcv no bytes to be written to master 
		hal_i2c_slave_rx(&i2c_handle,&slave_rcv_cmd,1);
  	while(i2c_handle.State != HAL_I2C_STATE_READY);
		
		hal_i2c_slave_tx(&i2c_handle,slave_tx_buffer,slave_rcv_cmd);
		while(i2c_handle.State != HAL_I2C_STATE_READY);

	}
  
#endif 
	
	//while(i2c_handle.State != HAL_I2C_STATE_READY);

}
	return 0;
}



/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 if(  ((uint16_t)0x0001) == GPIO_Pin)
 {
	// BSP_LED_On(LED5);
   TestReady = SET;
	 
	 
 }
}

void I2C1_ER_IRQHandler(void)
{
	HAL_I2C_ER_IRQHandler(& i2c_handle);
}


/**
  * @brief  This function handles I2C event interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to I2C data transmission
  */
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(& i2c_handle);
}



 