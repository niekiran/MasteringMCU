
/* Source code for PRINTF implementation over UART */
#include "uart_debug.h"
#include "hal_gpio_driver.h"
#include "hal_uart_driver.h"
#include <stdio.h>
#include <stdarg.h>

uart_handle_t debug_uart_handle;

/*this function initializes the debug uart */
void hal_debug_uart_init(uint32_t baudrate)
{
	gpio_pin_conf_t uart_pin_conf;
	
	/*Enable the clock for the GPIO PORT B */
	_HAL_RCC_GPIOB_CLK_ENABLE(); 
	
	/* USART3 is used , so enable the clock for the USART3 */
	_HAL_RCC_USART3_CLK_ENABLE();   
	
	debug_uart_handle.Instance          = USART_3;
	debug_uart_handle.Init.BaudRate     = DEBUG_USART_BAUD_9600;
	debug_uart_handle.Init.WordLength   = USART_WL_1S8B;
	debug_uart_handle.Init.StopBits     = UART_STOPBITS_1;
	debug_uart_handle.Init.Parity       = UART_PARITY_NONE;
	debug_uart_handle.Init.Mode         = UART_MODE_TX_RX;
	debug_uart_handle.Init.OverSampling = USART_OVER16_ENABLE;

	debug_uart_handle.tx_cmp_cb = 0;
	debug_uart_handle.rx_cmp_cb = 0;
	
	 hal_uart_init(&debug_uart_handle);
	
	/*configure the GPIO_PORT_B_PIN_10 for the TX functionality */
	uart_pin_conf.pin = DEBUG_UART_TX_PIN;
	uart_pin_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	uart_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	uart_pin_conf.speed =  GPIO_PIN_SPEED_HIGH;
	uart_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_set_alt_function(GPIOB,DEBUG_UART_TX_PIN,GPIO_AF_VALUE_USART2);
	hal_gpio_init(GPIOB ,&uart_pin_conf);

	/*configure the GPIO_PORT_B_PIN_11 for the RX functionality */
	uart_pin_conf.pin = DEBUG_UART_RX_PIN;
	hal_gpio_set_alt_function(GPIOB,DEBUG_UART_RX_PIN,GPIO_AF_VALUE_USART2);
	hal_gpio_init(GPIOB ,&uart_pin_conf);
	

}


/*this function implements printf over USART3 */
 void uart_printf(char *format,...)
 {
	char str[80];
	char *s;
	
	/*Extract the the argument list using VA apis */
	va_list args;
	va_start(args, format);
	vsprintf(str, format,args);
	s=str;

	/* until NULL, send out single byte in a blocking fashion */
	while(*s){
	// wait until data register is empty
	while( !(debug_uart_handle.Instance->SR & USART_REG_SR_TXE_FLAG) );
	debug_uart_handle.Instance->DR = *s;
	s++;
	}
	va_end(args);
}
	
