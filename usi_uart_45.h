/*
 * usi_uart_45.h
 *
 * USIUARTX - USI as UART in half-duplex mode
 *
 * @created: 2015-03-23
 * @author: Neven Boyanov
 * 2016-08-16
 * revised: Mark Malmros @chimneypoint
 * Original Source code available at: https://bitbucket.org/tinusaur/usiuartx
 *
 */

#ifndef _USI_UART_45_H
#define _USI_UART_45_H

// ============================================================================

#include <stdint.h>
#include <avr/io.h>


// ============================================================================

// REF: AVR307 Half Duplex UART Using the USI Module on tinyAVR and megaAVR devices

//********** USI UART Defines **********//

//#define SYSTEM_CLOCK    14745600
//#define SYSTEM_CLOCK    11059200
//#define SYSTEM_CLOCK     8000000
//#define SYSTEM_CLOCK     7372800
//#define SYSTEM_CLOCK     3686400
//#define SYSTEM_CLOCK     2000000
//#define SYSTEM_CLOCK     1843200
#define SYSTEM_CLOCK     1000000

//#define BAUDRATE 115200
//#define BAUDRATE  57600
//#define BAUDRATE  28800
//#define BAUDRATE  19200
//#define BAUDRATE  14400
//#define BAUDRATE   9600
//
//#define BAUDRATE   4800
#define BAUDRATE   2400
//#define BAUDRATE     1200

/* thi macro is only used to determine the TIMERO_SEED 
 * BUT does not actually set the prescaler
 * this must be done in the TCCR0B register (24A/44A/84A) bits CS0 [2:0] clock select in the .c file
 * see page 82 of the Attiny24A datasheet
 * the original code uses "no prescaler OR TIMER_PRESCALER 1 
 * IF SELECTING A DIFFERENT PRESCALER - ADJUST THE TCCR0B REGISTER ACCORDINGLY!
*/
//#define TIMER_PRESCALER  1
//
#define TIMER_PRESCALER  8

// Transmit and Receive buffer size
// Must be power of 2, i.e. 2,4,8,16,32,64,128,256
#define USIUARTX_RX_BUFFER_SIZE        32
#define USIUARTX_TX_BUFFER_SIZE        32

// ----------------------------------------------------------------------------

//********** USI_UART Prototypes **********//

void usiuartx_init(void);

uint8_t usiuartx_tx_has_data(void);
void usiuartx_tx_byte(uint8_t);
void usiuartx_tx_string(char *);

void usiuartx_rx_init(void);
uint8_t usiuartx_rx_has_data(void);
uint8_t usiuartx_rx_byte(void);

// ============================================================================

#endif
