
// this header file contains all the leds present on the board
# include "boards.h"

// To add the delay we need to inclue the delay header file
#include "nrf_delay.h"

// This is the function which initializes all the LEDs present on the board

// A function to initialize all the LEDs 
bsp_board_init(BSP_INIT_LEDS);

// We can also initialize all the Buttons along with the LEDs with the following flag included
bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS); // initializes the buttons as well as input



// For LEDS:
		
// Turns on the Led
bsp_board_led_on(0); // the parameter is simple integer ranging from 0 to 3 for LED1 to LED4 respectively

// Turns off the led
bsp_board_led_off(0); // the parameter is simple integer ranging from 0 to 3 for LED1 to LED4 respectively

// Same as toggle function
bsp_board_led_invert(0); // the parameter is simple integer ranging from 0 to 3 for LED1 to LED4 respectively


// to Turn on & turn off all the leds we use the following functions:
// Note: No parameter is needed in this function!!!
bsp_board_leds_on(); // Turn on all the LEDs

bsp_board_leds_off(); // Turn off all the LEDs


// For buttons 
// This function returns bool value true or false
  bsp_board_button_state_get(0);
  
// So we can use it like this:  
// Use this function to read the state of each button
  if(bsp_board_button_state_get(0) ) // The parameter is integer ranging from 0 to 3
  {
	  // Do your stuff here
	  
  }
  
  