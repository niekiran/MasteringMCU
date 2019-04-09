#ifndef __UART_MAIN_H
#define __UART_MAIN_H


//#define GPIO_AF7_USART1        ((uint8_t)0x07)  /* USART1 Alternate Function mapping     */
#define GPIO_AF7_USART2        ((uint8_t)0x07)  /* USART2 Alternate Function mapping     */
//#define GPIO_AF7_USART3        ((uint8_t)0x07)  /* USART3 Alternate Function mapping     */

#define GPIO_PIN_2_SEL                       2
#define GPIO_PIN_3_SEL                       3             

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2_SEL
#define USARTx_TX_GPIO_PORT              GPIOA  
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3_SEL
#define USARTx_RX_GPIO_PORT              GPIOA 
#define USARTx_RX_AF                     GPIO_AF7_USART2

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

#define EXTIx_IRQn                 EXTI0_IRQn
#define EXTIx_IRQHandler           EXTI0_IRQHandler

#define GPIO_BUTTON_PIN   0
#define GPIO_BUTTON_PORT  GPIOA




#endif 