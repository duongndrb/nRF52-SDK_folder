#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"

#define LED 17 
#define LED1 18
#define Button 13
#define ButtonE 16

/**
 * @brief Function for application main entry.
 */
int main(void)
{

    nrf_gpio_cfg_output(LED); // Configures the Led pin as Output pin
    nrf_gpio_cfg_output(LED1);

    nrf_gpio_range_cfg_input(13, 16, NRF_GPIO_PIN_PULLUP); // Configure the Button pin as Input pin

    nrf_gpio_pin_set(LED); //Turns off the LED
    nrf_gpio_pin_set(LED1);

    /* Toggle LEDs. */
    while (true)
    {
      
      if(nrf_gpio_pin_read(13) == 0)
      {
        nrf_gpio_pin_clear(LED); //Turn on the LED

        while(nrf_gpio_pin_read(13) == 0); //Stay in this loop until the button is released

        nrf_gpio_pin_set(LED); //Turns off the LED
      }

      if(nrf_gpio_pin_read(14) == 0)
      {
        nrf_gpio_pin_clear(LED1); //Turn on the LED

        while(nrf_gpio_pin_read(14) == 0); //Stay in this loop until the button is released

        nrf_gpio_pin_set(LED1); //Turns off the LED
      }
    }
}

/**
 *@}
 **/
