#ifndef __SPI_APP_C
#define __SPI_APP_C

#include<stdint.h>

/* Defines used for transfer communication */
#define CMD_MASTER_READ                               ((uint16_t)0x1234)
#define CMD_MASTER_WRITE                              ((uint16_t)0x5678)
#define CMD_LENGTH                                     2
#define DATA_LENGTH                                    4
#define ACK_LEN                                        2
#define SPI_ACK_BYTES                                  0xD5E5


/* Definition for SPIx's NVIC */
#define SPIx_IRQn                        SPI2_IRQn
#define SPIx_IRQHandler                  SPI2_IRQHandler

#define EXTIx_IRQn                 EXTI0_IRQn
#define EXTIx_IRQHandler           EXTI0_IRQHandler

/* Button details */
#define GPIO_BUTTON_PIN   0
#define GPIO_BUTTON_PORT  GPIOA

/* Macros used for Configuring gpios for SPI functionality */
#define GPIOB_PIN_13      13

#define GPIOB_PIN_14      14

#define GPIOB_PIN_15      15

#define SPI_CLK_PIN   GPIOB_PIN_13
#define SPI_MISO_PIN  GPIOB_PIN_14
#define SPI_MOSI_PIN  GPIOB_PIN_15

/* SPI alternate functionality value */
#define GPIO_PIN_AF5_SPI2   0x05


#endif 