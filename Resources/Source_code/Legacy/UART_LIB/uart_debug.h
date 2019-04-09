#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H
#include <stdint.h>
         
/* Definition for USARTx Pins */
#define DEBUG_UART_TX_PIN                10
#define DEBUG_UART_RX_PIN                11
  
  

#define GPIO_AF_VALUE_USART1         				((uint8_t)0x07) 
#define GPIO_AF_VALUE_USART2        				((uint8_t)0x07) 
#define GPIO_AF_VALUE_USART3a        				((uint8_t)0x07) 

#define DEBUG_USART_BAUD_9600                   (uint32_t)9600
#define DEBUG_USART_BAUD_115200                 (uint32_t)115200



void hal_debug_uart_init(uint32_t baudrate);
void uart_printf(char *format,...);
 
 
#endif