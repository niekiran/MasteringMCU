
#include<stdint.h>
#include "hal_gpio_driver.h"
#include "led.h"

/**
	* @brief  Initialize the LEDs 
	* @param  None
	* @retval None
	*/
void led_init(void)
{
	
	gpio_pin_conf_t led_pin_conf;
	/* enable the clock for the GPIOD port */
	_HAL_RCC_GPIOD_CLK_ENABLE();

	led_pin_conf.pin = LED_ORANGE;
	led_pin_conf.mode = GPIO_PIN_OUTPUT_MODE;
	led_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	led_pin_conf.speed =  GPIO_PIN_SPEED_VERY_HIGH;
	led_pin_conf.pull  =  GPIO_PIN_NO_PULL_PUSH;
	hal_gpio_init(GPIOD, &led_pin_conf);


	led_pin_conf.pin = LED_BLUE;
	hal_gpio_init(GPIO_PORT_D, &led_pin_conf);

	led_pin_conf.pin = LED_GREEN;
	hal_gpio_init(GPIO_PORT_D, &led_pin_conf);

	led_pin_conf.pin = LED_RED;
	hal_gpio_init(GPIO_PORT_D, &led_pin_conf);
}

/**
	* @brief  Turns ON the led which is connected on the given pin  
	* @param  *GPIOx : Base address of the GPIO Port
	* @param  Pin : pin number of the LED
	* @retval None
	*/
void led_turn_on(GPIO_TypeDef *GPIOx, uint16_t pin)
{
	hal_gpio_write_to_pin(GPIOx,pin, 1);
	
}

/**
	* @brief  Turns OFF the led which is connected on the given pin  
	* @param  *GPIOx : Base address of the GPIO Port
	* @param  Pin : pin number of the LED
	* @retval None
	*/
void led_turn_off(GPIO_TypeDef *GPIOx, uint16_t pin)
{
	hal_gpio_write_to_pin(GPIOx,pin, 0);
	
}

/**
	* @brief  Toggels the led which is connected on the given pin  
	* @param  *GPIOx : Base address of the GPIO Port
	* @param  Pin : pin number of the LED
	* @retval None
	*/
void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin)
{
	if(hal_gpio_read_from_pin(GPIOx,pin))
	{
		 hal_gpio_write_to_pin(GPIOx,pin, 0);
	}else
	{
		 hal_gpio_write_to_pin(GPIOx,pin, 1);
		
	}
	
}


