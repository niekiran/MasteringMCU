
#include "uart_main.h"
#include "hal_gpio_driver.h"
#include "hal_uart_driver.h"
#include "uart_debug.h"
#include "led.h"


uart_handle_t uart_handle, debug_handle;
void uart_gpio_init(void)
{
	gpio_pin_conf_t uart_pin_conf;
	
/*enable the clock for the GPIO port A */
	_HAL_RCC_GPIOA_CLK_ENABLE();  
	
/*configure the GPIO_PORT_A_PIN_2 as TX */
	uart_pin_conf.pin = USARTx_TX_PIN;
	uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	uart_pin_conf.speed =  GPIO_PIN_SPEED_HIGH;
	uart_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_set_alt_function(GPIOA,USARTx_TX_PIN,USARTx_TX_AF);
	hal_gpio_init(GPIOA ,&uart_pin_conf);

	/*configure the GPIO_PORT_A_PIN_3 as RX */
	uart_pin_conf.pin = USARTx_RX_PIN;
	hal_gpio_set_alt_function(GPIOA,USARTx_RX_PIN,USARTx_TX_AF);
	hal_gpio_init(GPIOA ,&uart_pin_conf);

}

uint8_t message1[] = "STM32F4xx Discovery board \n UART Sample App test\n June , 2016 \n";
uint8_t message2[] = "Invalid Command !!! \n";
uint8_t message3[] = "Success !! \n";
uint8_t rx_buffer[4];


void error_handler(void)
{
	 while(uart_handle.tx_state != HAL_UART_STATE_READY );
	 hal_uart_tx(&uart_handle,message2, sizeof(message2)-1);
	
}

void handle_cmd(int cmd, int led )
{
	if(cmd == 'H')
			{
				if(led == (int) 0xff )
				{
					led_turn_on(GPIOD,LED_ORANGE);
			 	 led_turn_on(GPIOD,LED_BLUE);
				 led_turn_on(GPIOD,LED_GREEN);
				 led_turn_on(GPIOD,LED_RED);
					
				}else
				{
				led_turn_on(GPIOD,led);
				}
				
				hal_uart_tx(&uart_handle,message3, sizeof(message3)-1);
			}else if (cmd == 'L')
			{
				if(led == (int) 0xff )
				{
					led_turn_off(GPIOD,LED_ORANGE);
			 	 led_turn_off(GPIOD,LED_BLUE);
				 led_turn_off(GPIOD,LED_GREEN);
				 led_turn_off(GPIOD,LED_RED);
					
				}else
				led_turn_off(GPIOD,led);
				
				hal_uart_tx(&uart_handle,message3, sizeof(message3)-1);
			} 
			else
			{
				error_handler();
			}
	
}

// this function parses the command and takes action 
void 	parse_cmd(uint8_t *cmd)
{
	
	if( cmd[0] == 'L' && cmd[1] == 'E' && cmd[2] == 'D' )
	{
		if(cmd[3] == 'O' )
		{
			handle_cmd(cmd[4],LED_ORANGE);
			
		}else if(cmd[3] == 'B' )
		{
			handle_cmd(cmd[4],LED_BLUE);
			
		}else if(cmd[3] == 'G' )
		{
				handle_cmd(cmd[4],LED_GREEN);
			
		}else if(cmd[3] == 'R' )
		{
	    	handle_cmd(cmd[4],LED_RED);
		}else if (cmd[3] == 'A' )
		{
			handle_cmd(cmd[4],0xff);
		}
		else 
		{
			;
		}
		
		
		
	}else
	{
		error_handler();
		
	}
	
}

/*This callback will be called by the driver when driver finishes the transmission of data */
void app_tx_cmp_callback(void *size)
{
 
	
}

/*This callback will be called by the driver when the application receives the command */
void app_rx_cmp_callback(void *size)
{
	//we got a command,  parse it 
	parse_cmd(rx_buffer);
	
}


int main(void)
{
	
	uint32_t val;

	/* Led inits */	
	led_init();

	/*configures GPIOs for UART functionality */
	uart_gpio_init();
	
	/*enable the clock for the USART2 Peripheral */
	_HAL_RCC_USART2_CLK_ENABLE();   
	
	uart_handle.Instance          = USART_2;

	uart_handle.Init.BaudRate     = USART_BAUD_9600;
	uart_handle.Init.WordLength   = USART_WL_1S8B;
	uart_handle.Init.StopBits     = UART_STOPBITS_1;
	uart_handle.Init.Parity       = UART_PARITY_NONE;
	uart_handle.Init.Mode         = UART_MODE_TX_RX;
	uart_handle.Init.OverSampling = USART_OVER16_ENABLE;

/*fill out the application callbacks */
	uart_handle.tx_cmp_cb = app_tx_cmp_callback;
	uart_handle.rx_cmp_cb = app_rx_cmp_callback;
	
	 hal_uart_init(&uart_handle);

/*enable the IRQ of USART2 peripheral */
	 NVIC_EnableIRQ(USARTx_IRQn);

  while(uart_handle.tx_state != HAL_UART_STATE_READY );
	/*Send the message */
	//uint8_t message1[] = "STM32F4xx Discovery board \n UART Sample App test\n June , 2016 \n";
	hal_uart_tx(&uart_handle,message1, sizeof(message1)-1);

while(1)
{
  while(uart_handle.rx_state != HAL_UART_STATE_READY );
	/*receive the message */
	hal_uart_rx(&uart_handle,rx_buffer, 5 );
}

	return 0;
}

//This is the ISR for the USARTx interrupt , defined in the hal_uart_driver.h
void USARTx_IRQHandler(void)
{
  hal_uart_handle_interrupt(&uart_handle);
}
