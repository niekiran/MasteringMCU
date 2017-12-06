#ifndef __I2C_MAIN_H
#define __I2C_MAIN_H



#define GPIOB_PIN_6                      6
#define GPIOB_PIN_9                      9
#define I2C1_SCL_LINE                   GPIOB_PIN_6
#define I2C1_SDA_LINE                   GPIOB_PIN_9

/* Definition for I2Cx's NVIC */
#define I2Cx_EV_IRQn                    I2C1_EV_IRQn
#define I2Cx_EV_IRQHandler              I2C1_EV_IRQHandler
#define I2Cx_ER_IRQn                    I2C1_ER_IRQn
#define I2Cx_ER_IRQHandler              I2C1_ER_IRQHandler

#define ALT_FUN_4                       0x04
#define GPIO_PIN_AF4_I2C123             ALT_FUN_4
#endif 