
// Include this in your project
#include "nrf_gpio.h"




// Function to Configure the Pin as INPUT pin

/*
1st Parameter:

pin_number can be equal to:
0 to 31 // (number)


2nd Parameter:

 pull_config can be equal to any of these values:

 NRF_GPIO_PIN_PULLUP
 NRF_GPIO_PIN_PULLDOWN
 NRF_GPIO_PIN_NOPULL
 
 for e.g:
 nrf_gpio_cfg_input(13,NRF_GPIO_PIN_NOPULL); // Configure Pin 13 as input with no internal pull_up resistor

 
*/
 nrf_gpio_cfg_input( uint32_t pin_number,
					 nrf_gpio_pin_pull_t pull_config);





/*
for e.g: 

	nrf_gpio_range_cfg_input (13,16，NRF_GPIO_PIN_PULLUP);

*/

// Function to configure multiple consective pins as input pins:
nrf_gpio_range_cfg_input ( uint32_t pin_range_start,
						   uint32_t pin_range_end,
						   nrf_gpio_pin_pull_t pull_config )






/*

*/

// Function to Read Input Pin value:
nrf_gpio_pin_read(uint32_t pin_number);



// Function to read all 32 pins status i.e read all pins at once from pin 0 to pin31 all together. Not usually needed
nrf_gpio_port_in_read();