
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "nrf_uart.h"

#define UART_TX_BUFF_SIZE 128
#define UART_RX_BUFF_SIZE 128

// Hardware flow control
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED


/**
 * @brief Function for main application entry.
 */

//Error handle
// A simple error handler for uart if something goes wrong...
void uart_err_handle(app_uart_evt_type_t * p)
{
  
}


int main(void)
{
  uint32_t err_code; // a variable to hold the error value
  
  bsp_board_init(BSP_INIT_LEDS); // initialize leds

  const app_uart_comm_params_t com_params = 
  {
    RX_PIN_NUMBER, 
    TX_PIN_NUMBER,
    RTS_PIN_NUMBER,
    CTS_PIN_NUMBER,
    UART_HWFC, // hardware flow control disabled
    false, // parity = none
    NRF_UART_BAUDRATE_115200
  };

  // pass all the values to this function to initialize the UART module
  APP_UART_FIFO_INIT(&com_params, UART_RX_BUFF_SIZE, UART_TX_BUFF_SIZE, uart_err_handle,APP_IRQ_PRIORITY_LOWEST, err_code);


  
  APP_ERROR_CHECK(err_code); // check if everything initialized correctly
  // Print a start msg over uart port
  printf("Hello PC from nordic Device!! \r\n");

  while(true)
  {
    uint8_t cr; // create a variable to hold the UART char data
    while(app_uart_get(&cr)!= NRF_SUCCESS); //wait here until it receives characters from pc 
    if(cr == 't') // check if the received char is t
    {
      bsp_board_leds_on(); // turn on all the leds
      printf("Leds Turned On!! \r\n"); // print out the state of leds
    }
    if(cr == 'k') // check if the received char is k
    {
      bsp_board_leds_off(); // turn off all the leds
      printf("Leds Turned Off!! \r\n");// print out the state of leds

    }
    
  }// while loop closed

} // main function closed


/** @} */
