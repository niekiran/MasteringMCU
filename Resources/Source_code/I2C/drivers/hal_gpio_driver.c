#include <stdint.h>
#include "hal_gpio_driver.h"

INT_CALLBCK callback_ptr;

void hal_gpio_configure_pin_mode(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t mode)
{
	 GPIOx->MODER   |= (mode << ( 2 * pin_no));
}


void hal_gpio_configure_pin_speed(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t speed)
{
	
	 GPIOx->OSPEEDR |= (speed << (2 * pin_no));
}


void hal_gpio_configure_pin_otype(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t op_type)
{
	
	 GPIOx->OTYPER |= (op_type << pin_no);
	
}

void hal_gpio_configure_pin_pupd(GPIO_TypeDef *GPIOx, uint16_t pin_no, uint32_t pupd)
{
	 GPIOx->PUPDR |= (pupd << (2 * pin_no));

}


void hal_gpio_init(GPIO_TypeDef *GPIOx, gpio_pin_conf_t *gpio_pin_conf)
{
	hal_gpio_configure_pin_mode(GPIOx,gpio_pin_conf->pin, gpio_pin_conf->mode);
	hal_gpio_configure_pin_speed(GPIOx,gpio_pin_conf->pin, gpio_pin_conf->speed);
	hal_gpio_configure_pin_otype(GPIOx,gpio_pin_conf->pin, gpio_pin_conf->op_type);
	hal_gpio_configure_pin_pupd(GPIOx,gpio_pin_conf->pin, gpio_pin_conf->pull);
	
}

void hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no, uint8_t val)
{
	if(val)
		GPIOx->ODR |=  (1 << pin_no);
	else
		GPIOx->ODR &=  ~(1 << pin_no);
	
}

uint8_t hal_gpio_read_from_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no)
{
	
	uint8_t value ; 
	
	value = ((GPIOx->IDR >> pin_no ) & 0x00000001 );
	
	return value ;
}

void hal_gpio_set_alt_function(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint16_t alt_fun_value)
{
  if ( pin_no  <= 7 )
	{
		GPIOx->AFR[0] |= (alt_fun_value << ((pin_no) * 4 ));
	}
	else
	{
		GPIOx->AFR[1] |= (alt_fun_value << ( ( pin_no % 8) * 4 ));
	}
}	


void hal_gpio_configure_interrupt(uint16_t pin_no, int_edge_sel_t edge_sel, INT_CALLBCK isr)
{
	callback_ptr = isr;
	
	if (edge_sel == INT_RISING_EDGE )
	{
		EXTI->RTSR |= 1 << pin_no;
	}
	else if (edge_sel == INT_FALLING_EDGE)
	{
		EXTI->FTSR |= 1 << pin_no;
	}else if (edge_sel == INT_RISING_FALLING_EDGE )
	{
		EXTI->FTSR |= 1 << pin_no;
		EXTI->RTSR |= 1 << pin_no;
	}
	else
	{
		 ;//TODO
	}
	
}
void hal_gpio_enable_interrupt(uint16_t pin_no)
{
	EXTI->IMR |= 1 << pin_no;
	NVIC_EnableIRQ(EXTI0_IRQn);
}

void 	hal_gpio_clear_interrupt(uint16_t pin)
{
	EXTI->PR |= 1 << pin;
	
}

void EXTI0_IRQHandler(void)
{
	if(callback_ptr)
		
	callback_ptr();
}
