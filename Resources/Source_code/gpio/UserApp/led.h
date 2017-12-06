#ifndef __LED_H
#define __LED_H

#include "hal_gpio_driver.h"

/* LEDs GPIO definitions */

#define GPIOD_PIN_12  12
#define GPIOD_PIN_13  13
#define GPIOD_PIN_14  14
#define GPIOD_PIN_15  15

#define LED_GREEN     GPIOD_PIN_12
#define LED_ORANGE    GPIOD_PIN_13
#define LED_RED       GPIOD_PIN_14
#define LED_BLUE      GPIOD_PIN_15

#define GPIO_BUTTON_PIN 0

/**
* @brief  Initialize the LEDs 
* @param  None
* @retval None
*/
void led_init(void);

/**
* @brief  Turns ON the led which is connected on the given pin  
* @param  *GPIOx : Base address of the GPIO Port
* @param  Pin : pin number of the LED
* @retval None
*/
void led_turn_on(GPIO_TypeDef *GPIOx, uint16_t pin);

/**
* @brief  Turns OFF the led which is connected on the given pin  
* @param  *GPIOx : Base address of the GPIO Port
* @param  Pin : pin number of the LED
* @retval None
*/
void led_turn_off(GPIO_TypeDef *GPIOx, uint16_t pin);

/**
* @brief  Toggels the led which is connected on the given pin  
* @param  *GPIOx : Base address of the GPIO Port
* @param  Pin : pin number of the LED
* @retval None
*/
void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin);

#endif 