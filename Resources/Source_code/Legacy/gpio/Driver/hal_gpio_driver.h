#ifndef HAL_GPIO_DRIVER_H
#define HAL_GPIO_DRIVER_H

/* MC specific headerfile for stm32f407vgt6 based Discovery board */
#include "stm32f407xx.h"

/******************************************************************************/
/*                                                                            */
/*                     1. Macros used for GPIO pin Initialization               */
/*                                                                            */
/******************************************************************************/
/* GPIO Mode Settings values*/
#define GPIO_PIN_INPUT_MODE                            ( (uint32_t)0x00 )
#define GPIO_PIN_OUTPUT_MODE                           ( (uint32_t)0x01)
#define GPIO_PIN_ALT_FUN_MODE                          ( (uint32_t)0x02)

/* GPIO OP type selection values */
#define GPIO_PIN_OP_TYPE_PUSHPULL                      ( (uint32_t)0x00 )
#define GPIO_PIN_OP_TYPE_OPEN_DRAIN                    ( (uint32_t)0x01 )

/* GPIO Speed type selection values */
#define GPIO_PIN_SPEED_LOW                             ( (uint32_t)0x00 )
#define GPIO_PIN_SPEED_MEDIUM                          ( (uint32_t)0x01 )
#define GPIO_PIN_SPEED_HIGH                            ( (uint32_t)0x02 )
#define GPIO_PIN_SPEED_VERY_HIGH                       ( (uint32_t)0x03 )

/* GPIO pull up/pull dwn  selection values */
#define GPIO_PIN_NO_PULL_PUSH                          ( (uint32_t)0x00 )
#define GPIO_PIN_PULL_UP                               ( (uint32_t)0x01 )
#define GPIO_PIN_PULL_DOWN                             ( (uint32_t)0x02 )


/* gpio port address */

#define GPIO_PORT_A  GPIOA
#define GPIO_PORT_B  GPIOB
#define GPIO_PORT_C	 GPIOC
#define GPIO_PORT_D  GPIOD
#define GPIO_PORT_E  GPIOE
#define GPIO_PORT_F  GPIOF
#define GPIO_PORT_G  GPIOG
#define GPIO_PORT_H  GPIOH
#define GPIO_PORT_I  GPIOI

/* Macros to Enable Clock for diffrent GPIO ports in RCC register */

#define _HAL_RCC_GPIOA_CLK_ENABLE()       (RCC->AHB1ENR |=  (1 << 0) )
#define _HAL_RCC_GPIOB_CLK_ENABLE()       (RCC->AHB1ENR |=  (1 << 1) )
#define _HAL_RCC_GPIOC_CLK_ENABLE()       (RCC->AHB1ENR |=  (1 << 2) )
#define _HAL_RCC_GPIOD_CLK_ENABLE()       (RCC->AHB1ENR |=  (1 << 3) )
#define _HAL_RCC_GPIOE_CLK_ENABLE()       (RCC->AHB1ENR |=  (1 << 4) )
#define _HAL_RCC_GPIOF_CLK_ENABLE()       (RCC->AHB1ENR |=  (1 << 5) )
#define _HAL_RCC_GPIOG_CLK_ENABLE()       (RCC->AHB1ENR |=  (1 << 6) )
#define _HAL_RCC_GPIOH_CLK_ENABLE()       (RCC->AHB1ENR |=  (1 << 7) )


/******************************************************************************/
/*                                                                            */
/*                      Data Structure for GPIO pin Initialization            */
/*                                                                            */
/******************************************************************************/

/**
* @brief  GPIO pin configuration structure
*         This structure will be filled and passed to driver by the applications to 
*         initialize the gpio pin
*/
typedef struct
{
	uint32_t pin;            /*Specifies the GPIO pins to be configured */

	uint32_t mode;           /*Specifies the operating mode for the selected pins */
													
	uint32_t op_type;        /*Specifies the output type for the selected pins*/

	uint32_t pull;           /*Specifies the Pull-up or Pull-Down activation for the selected pins */
							
	uint32_t speed;          /* Specifies the speed for the selected pins */
							 
	uint32_t alternate;      /*Specifies the alternate function value, 
											 if the mode is set for alt function mode */                      
}gpio_pin_conf_t;


/**
* @brief  Interrupt Edge selection enum 
*/
typedef enum
{
	INT_RISING_EDGE,
	INT_FALLING_EDGE,
	INT_RISING_FALLING_EDGE
}int_edge_sel_t;

/******************************************************************************/
/*                                                                            */
/*                      Driver exposed APIs                                   */
/*                                                                            */
/******************************************************************************/
/**
	* @brief  Initializes the gpio pin 
	* @param  *GPIOx : GPIO Port Base address
	* @param  *gpio_pin_conf :Pointer to the pin conf structure sent by application 
	* @retval None
	*/
void hal_gpio_init(GPIO_TypeDef *GPIOx, gpio_pin_conf_t *gpio_pin_conf);

/**
	* @brief  Read a value from a  given pin number 
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @retval uint8_t: Value read 
	*/
uint8_t hal_gpio_read_from_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no);

/**
	* @brief  Write a value to given pin number 
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  value   : value to be written 
	* @retval None
	*/
void hal_gpio_write_to_pin(GPIO_TypeDef *GPIOx,uint16_t pin_no, uint8_t val);

/**
	* @brief  Set the alternate functionality for the given pin  
	* @param  *GPIOx : GPIO Port Base address
	* @param  pin_no : GPIO pin number 
	* @param  alt_fun_value   :  alternate function to be configured with 
	* @retval None
	*/
void hal_gpio_set_alt_function(GPIO_TypeDef *GPIOx,uint16_t pin_no,uint16_t alt_fun_value);

/**
	* @brief  Configure the interrupt for a given pin number   
	* @param  pin_no : GPIO pin number 
	* @param  edge_sel   :  Triggering edge slection value of type "int_edge_sel_t"
	* @retval None
	*/
void hal_gpio_configure_interrupt(uint16_t pin_no, int_edge_sel_t edge_sel);

/**
	* @brief  Enable the interrupt for a give pin number and irq number  
	* @param  pin_no : GPIO pin number 
	* @param  irq_no   :  irq_number to be enabled in NVIC 
	* @retval None
	*/
void hal_gpio_enable_interrupt(uint16_t pin_no, IRQn_Type irq_no);

/**
	* @brief  Clear the sticky interrupt pending bit if set 
	* @param  pin_no : GPIO pin number 
	* @retval None
	*/
void 	hal_gpio_clear_interrupt(uint16_t pin);


#endif 
