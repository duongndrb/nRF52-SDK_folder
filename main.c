
#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"


/**
 * @brief Function for application main entry.
 */
int main(void)
{
   bsp_board_init(BSP_INIT_LEDS); // Initialize all the LEDs available on the board

   while(true) // forever loop
   {
	   
	   
     bsp_board_led_on(0); // Turn on the first Led ( Set the LED pin to Logic LOW Level)
     nrf_delay_ms(500); // Delay for 500 ms
     bsp_board_led_off(0); // Turn off the first Led ( Set the LED pin to Logic HIGH Level)
     nrf_delay_ms(500); // Delay for 500 ms




     bsp_board_led_invert(2); // A function to toggle the state of respective led( LED 3 in this case)
     nrf_delay_ms(100); // Delay for 100 ms




     bsp_board_leds_on(); // Turn on all the LEDs
     nrf_delay_ms(50);
     bsp_board_leds_off(); // Turn off all the LEDs
     nrf_delay_ms(50);
     
      
   }
}

/**
 *@}
 **/
