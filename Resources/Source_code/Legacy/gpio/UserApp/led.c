
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
	led_pin_conf.speed =  GPIO_PIN_SPEED_MEDIUM;
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
	
#if 0
	//Logic 2
	hal_gpio_write_to_pin(GPIOx,pin, ~(hal_gpio_read_from_pin(GPIOx,pin)));
#endif 
}

int main(void)
{
	uint32_t i;

	/* Initializes the LEDs */
	led_init();
	
	/* Enable the clock for the GPIOA Port */
	_HAL_RCC_GPIOA_CLK_ENABLE();
	
		//set the mode as input
	GPIOA->MODER &= ~0x3;
	GPIOA->PUPDR  &= ~0x3;
		//enable clock for RCC
	RCC->APB2ENR |= 0x00004000;
	/*Configure the button interrupt as falling edge */
	hal_gpio_configure_interrupt(GPIO_BUTTON_PIN, INT_RISING_FALLING_EDGE);
	/*Enable the interrupt on EXTI0 line */
	hal_gpio_enable_interrupt(GPIO_BUTTON_PIN,EXTI0_IRQn);

#if 1
	while(1)
	{
		led_turn_on(GPIOD,LED_ORANGE);
		led_turn_on(GPIOD,LED_BLUE);

		for(i=0;i<500000;i++);

		led_turn_off(GPIOD,LED_ORANGE);
		led_turn_off(GPIOD,LED_BLUE);

		for(i=0;i<500000;i++);

	}

#endif 
	
	while(1);
	
}

/**
	* @brief  ISR for the configured EXTI0 interrupt  
	* @retval None
	*/
void EXTI0_IRQHandler(void)
{
  hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	/* Do youR TASK here */
	led_toggle(GPIOD,LED_BLUE);
	led_toggle(GPIOD,LED_ORANGE);
	led_toggle(GPIOD,LED_RED);
	led_toggle(GPIOD,LED_GREEN);
}

