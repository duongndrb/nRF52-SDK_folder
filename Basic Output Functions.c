

// Include this in your project
#include "nrf_gpio.h"




// Function to Configure the Pin as OUTPUT pin
 nrf_gpio_cfg_output(uint32_t pin_number);





//Function to Set the pin to Logic High State i.e. 1
nrf_gpio_pin_set(uint32_t pin_number);





//Function to Set the pin to Logic Low State i.e. 0
nrf_gpio_pin_reset(uint32_t pin_number);






// Function to inverse the pin state i.e. if state = 1 change it to 0  & if state = 0 then change it to 1
nrf_gpio_pin_toggle(uint32_t pin_number);




// Function to configure Multiple pins as output pins for same configuration
nrf_gpio_range_cfg_output (uint32_t start_pin_number,uint32_t end_pin_number);



/*

This is the configuration function which configures all the internal parameters iteself.

STATIC_INLINE void nrf_gpio_cfg (
 uint32_t pin_number,
 nrf_gpio_pin_dir_t dir,
 nrf_gpio_pin_input_t,
 nrf_gpio_pin_pull_t pull,
 nrf_gpio_pin_drive_t drive,
 nrf_gpio_pin_sense_t sense
)


*/


















