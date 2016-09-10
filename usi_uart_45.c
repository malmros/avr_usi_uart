/*
 * usi_uart_45.c
 *
 * USIUARTX - USI as UART in half-duplex mode
 *
 * @created: 2015-03-23
 * @author: Neven Boyanov
 * reconfigured: 2016-08-16
 * revised: Mark Malmros @chimneypoint
 *
 * Original Source code available at: https://bitbucket.org/tinusaur/usiuartx
 * NOTE changed various registers & bit defs  to compile/run with 45
 */

#define F_CPU 1000000UL		  // Sets up the default speed for delay.h

#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include "usi_uart_45.h"

// ============================================================================

// REF: AVR307 Half Duplex UART Using the USI Module on tinyAVR and megaAVR devices

// #include <ioavr.h>  // Note there is a bug in ioavr.h file that includes iotiny22.h instead of iotiny26.h.
// #include <iotiny26.h>
// #include <inavr.h>
// #include "USI_UART_config.h"

// ----------------------------------------------------------------------------

// ATtiny26  USI pins are DO : PB1   and DI : PB0
// Attiny24a/44a/84a  USI pins are DO : PA5 (pin 8)  and DI : PA6 (pin 7)
/*****************************************************************************/
// ATtiny45  USI pins are 	DO: PB1 ( physical pin 5) :::PCINT1    (uart data out)
//							D1: PB0 (physical pin 6) ::: PCINT0    (uart data in)

//********** USI UART Defines **********//

#define DATA_BITS                 8
#define START_BIT                 1
#define STOP_BIT                  1
#define HALF_FRAME                5

#define USI_COUNTER_MAX_COUNT     16
#define USI_COUNTER_SEED_TRANSMIT (USI_COUNTER_MAX_COUNT - HALF_FRAME)
#define INTERRUPT_STARTUP_DELAY   (0x11 / TIMER_PRESCALER) // ?? HOW??
// TIMER0_SEED_COMPENSATION: timer0 seed compensation (for interrupt latency).
// Ex.: ESP8266 + ATtiny85/Tinusaur = 7,8
// THIS IS EITHER 7,8, OR 9 SEE PAGE 6 OF AVR307 doc
#define TIMER0_SEED_COMPENSATION  4 // 0, 1, 2, 3, 4, 5, 6, 7, 8 ...as in the mikrocontroller version
#define TIMER0_SEED               (256 - ( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ))
// + TIMER0_SEED_COMPENSATION  see note in ../usiuartx/USI_UART.c
// NOTE NO TIMER0_SEED_COMPENSATION IS USED HERE!!!
#if ( (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 3/2) > (256 - INTERRUPT_STARTUP_DELAY) )
    #define INITIAL_TIMER0_SEED       ( 256 - (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 1/2) )
    #define USI_COUNTER_SEED_RECEIVE  ( USI_COUNTER_MAX_COUNT - (START_BIT + DATA_BITS) )
#else
    #define INITIAL_TIMER0_SEED       ( 256 - (( (SYSTEM_CLOCK / BAUDRATE) / TIMER_PRESCALER ) * 3/2) )
    #define USI_COUNTER_SEED_RECEIVE  (USI_COUNTER_MAX_COUNT - DATA_BITS)
#endif

#define USIUARTX_RX_BUFFER_MASK ( USIUARTX_RX_BUFFER_SIZE - 1 )
#if ( USIUARTX_RX_BUFFER_SIZE & USIUARTX_RX_BUFFER_MASK )
    #error RX buffer size is not a power of 2
#endif

#define USIUARTX_TX_BUFFER_MASK ( USIUARTX_TX_BUFFER_SIZE - 1 )
#if ( USIUARTX_TX_BUFFER_SIZE & USIUARTX_TX_BUFFER_MASK )
    #error TX buffer size is not a power of 2
#endif

/* General defines */
#define TRUE                      1
#define FALSE                     0

// ----------------------------------------------------------------------------

//********** Static Variables **********//

// __no_init __regvar static unsigned char usiuartx_tx_data @ 15;   // Tells the compiler to store the byte to be transmitted in registry.
static uint8_t usiuartx_tx_data;
//static volatile unsigned char USI_UART_TxData;
// see note in ../usiuartx/USI_UART.c
// OR...
//register uint8_t usiuartx_tx_data asm("r15");				// Tells the compiler to use Register 15 instead of SRAM

static uint8_t usiuartx_rx_buffer[USIUARTX_RX_BUFFER_SIZE];	// UART transmit buffer. Size is definable in the header file.
static volatile uint8_t usiuartx_rx_head;	// Index pointing at the beginning (the head) of the transmit buffer.
static volatile uint8_t usiuartx_rx_tail;	// Index pointing at the end (the tail) of the transmit buffer.
static uint8_t usiuartx_tx_buffer[USIUARTX_TX_BUFFER_SIZE];	// UART receive buffer. Size is definable in the header file.
static volatile uint8_t usiuartx_tx_head;	// Index pointing at the beginning (the head) of the receive buffer.
static volatile uint8_t usiuartx_tx_tail;	// Index pointing at the end (the tail) of the receive buffer.

static volatile union USI_UART_status                           // Status byte holding flags.
{
    uint8_t status;
    struct
    {
        uint8_t ongoing_Transmission_From_Buffer:1;
        uint8_t ongoing_Transmission_Of_Package:1;
        uint8_t ongoing_Reception_Of_Package:1;
        uint8_t reception_Buffer_Overflow:1;
        uint8_t flag4:1;
        uint8_t flag5:1;
        uint8_t flag6:1;
        uint8_t flag7:1;
    };
} USI_UART_status = { 0 };

// ----------------------------------------------------------------------------

uint8_t usiuartx_bit_reverse(uint8_t);
void usiuartx_rx_init(void);
void usiuartx_tx_init(void);

// ----------------------------------------------------------------------------

//********** USI_UART functions **********//

// Reverses the order of bits in a byte.
// I.e. MSB is swapped with LSB, etc.
uint8_t usiuartx_bit_reverse( uint8_t x )
{
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;
}

// Initialize.
void usiuartx_init(void)
{
	// Reset the buffers - that is to clear the indexes.
    usiuartx_rx_tail = 0;
    usiuartx_rx_head = 0;
    usiuartx_tx_tail = 0;
    usiuartx_tx_head = 0;
	
	// Initialisation for USI_UART receiver
    usiuartx_rx_init();
}

// ----------------------------------------------------------------------------

// Initialise USI for UART transmission.
void usiuartx_tx_init(void)
{
    cli();	// __disable_interrupt();
    TCNT0  = 0x00;	// Counter0 set to 0
	GTCCR = (1  << TSM ) | ( 1<< PSR0 ); // see page 114 of 24A doc
//	TCCR0B = (0<<CS02)|(0<<CS01)|(1<<CS00);  			      // Reset the pre-scaler and start Timer0.  these bit are for TIMER_PRESCALER 1
	TCCR0B = (0<<CS02)|(1<<CS01)|(0<<CS00);  			      // Reset the pre-scaler and start Timer0.  these bitS are for TIMER_PRESCALER 8
// changed from 	TCCR0B = (1<<PSR0)|(0<<CS02)|(0<<CS01)|(1<<CS00);
// to reset the prescaler, another register is used in the attiny44a
	GTCCR = (0 << TSM ); // see note on this reg above
//	GTCCR =  (1 << PSR10); // presummably (?) this is the same as 1<<PSR0 in the above register (for the 26)!!............(1 << TSM) |
    TIFR   = (1<<TOV0);                                       // Clear Timer0 OVF interrupt flag.
//   TIFR ... change mk
    TIMSK |= (1<<TOIE0);                                      // Enable Timer0 OVF interrupt.
//   TIMSK ... change mkm
    USICR  = (0<<USISIE)|(1<<USIOIE)|                         // Enable USI Counter OVF interrupt.
             (0<<USIWM1)|(1<<USIWM0)|                         // Select Three Wire mode.
             (0<<USICS1)|(1<<USICS0)|(0<<USICLK)|             // Select Timer0 OVER as USI Clock source. SEE PAGE 6 OF AVR307 doc
             (0<<USITC);

    USIDR  = 0xFF;                                            // Make sure MSB is '1' before enabling USI_DO.
    USISR  = 0xF0 |                                           // Clear all USI interrupt flags.
//	sets last or upper 4 bits to 1's and lower 4 bits to 0's
//	try clearing ALL bits? i.e 0x00 ???
             0x0F;                                            // Pre-load the USI counter to generate interrupt at first USI clock.
// same register in 26 and 24 ...
    DDRB  |= (1<<PB1);                                        // Configure USI_DO as output.
//    DDRA  |= (1<<PA5);                                        // Configure USI_DO as output.
/* the pin configuration for the 44a/84a has PA5 (pin 8) as USI_DO and ...
 * PA6 (pin 7) as USI_DI   since we are using the USI ... adjust accordingly!
*/
    USI_UART_status.ongoing_Transmission_From_Buffer = TRUE;

    sei();	// __enable_interrupt();
}

#define USIUARTX_TX_ISEMPTY() (usiuartx_tx_head == usiuartx_tx_tail)
#define USIUARTX_TX_HASDATA() (usiuartx_tx_head != usiuartx_tx_tail)

// Check if there is data in the transmit buffer.
uint8_t usiuartx_tx_has_data(void)
{
    return (USIUARTX_TX_HASDATA());	// Return 0 (FALSE) if the transmit buffer is empty.
    // return (usiuartx_tx_head != usiuartx_tx_tail);
}

// Puts data in the transmission buffer, after reversing the bits in the byte.
// Initiates the transmission routines if not already started.
void usiuartx_tx_byte(uint8_t data)
{
	// DEBUGGING_CHAR(data);
    uint8_t tmp_tx_head;
    tmp_tx_head = ( usiuartx_tx_head + 1 ) & USIUARTX_TX_BUFFER_MASK;	// Calculate buffer index.
	// DEBUGGING_VARU("tmp_tx_head", tmp_tx_head);
    while ( tmp_tx_head == usiuartx_tx_tail );	// Wait for free space in buffer.
    usiuartx_tx_buffer[tmp_tx_head] = usiuartx_bit_reverse(data);	// Reverse the order of the bits in the data byte and store data in buffer.
    usiuartx_tx_head = tmp_tx_head;	// Store new index.
    if ( !USI_UART_status.ongoing_Transmission_From_Buffer )	// Start transmission from buffer (if not already started).
    {
        while ( USI_UART_status.ongoing_Reception_Of_Package );	// Wait for USI to finish reading incoming data.
        usiuartx_tx_init();
    }
}

void usiuartx_tx_string(char *text)
{
	// DEBUGGING_STRINGLN(text);
	while (*text)
	{
		usiuartx_tx_byte(*text++);
	}
}

// ----------------------------------------------------------------------------

// Initialise USI for UART reception.
// Note that this function only enables pinchange interrupt on the USI Data Input pin.
// The USI is configured to read data within the pinchange interrupt.
void usiuartx_rx_init(void)
{

    PORTB |=   (1<<PB1)|(1<<PB0);                           // Enable pull up on USI DO, DI pins. (USCK/PB2 and PB3 not needed)
    DDRB  &= ~((1<<PB1)|(1<<PB0));                          // Set USI DI, DO pins as inputs. (USCK/PB2 and PB3 not needed)
/*
    PORTA |=   (1<<PA6)|(1<<PA5);                           // Enable pull up on USI DO, DI pins.
    DDRA  &= ~((1<<PA6)|(1<<PA5));                          // Set USI DI, DO pins as inputs.
// change mkm  DO is PA5 , DI is PA6
*/
    USICR  =  0;                                            // Disable USI.
    GIFR   =  (1<<PCIF);                                    // Clear pin change interrupt flag.
//                PCIF ... change mkm  OK
    GIMSK |=  (1<<PCIE);                                    // Enable pin change interrupt for PCINT[5:0]
//................PCIE ... change mkm  OK
    PCMSK |=  (1<<PCINT0);									// Pin Change Mask Register, enable for PCINT0/PB0
//   PCMSK0 |=  (1<<PCINT0); ... change mkm
}

#define USIUARTX_RX_ISEMPTY() (usiuartx_rx_head == usiuartx_rx_tail)
#define USIUARTX_RX_HASDATA() (usiuartx_rx_head != usiuartx_rx_tail)

// Check if there is data in the receive buffer.
uint8_t usiuartx_rx_has_data(void)
{
    // return (usiuartx_rx_head != usiuartx_rx_tail);
    return (USIUARTX_RX_HASDATA());	// Return 0 (FALSE) if the receive buffer is empty.
}

// Returns a byte from the receive buffer. Waits if buffer is empty.
uint8_t usiuartx_rx_byte(void)
{
    uint8_t tmp_rx_tail;
    // while (usiuartx_rx_head == usiuartx_rx_tail);
    while (USIUARTX_RX_ISEMPTY());	// Wait for incoming data
    tmp_rx_tail = ( usiuartx_rx_tail + 1 ) & USIUARTX_RX_BUFFER_MASK;	// Calculate buffer index
    usiuartx_rx_tail = tmp_rx_tail;	// Store new index
    return usiuartx_bit_reverse(usiuartx_rx_buffer[tmp_rx_tail]);	// Reverse the order of the bits in the data byte before it returns data from the buffer.
}

// ----------------------------------------------------------------------------

// ********** Interrupt Handlers ********** //

// The pin change interrupt is used to detect USI_UART reception.
// It is here the USI is configured to sample the UART signal.
// #pragma vector=IO_PINS_vect
// __interrupt void IO_Pin_Change_ISR(void)
ISR(PCINT0_vect)
// change from  ISR(SIG_PIN_CHANGE)  mkm
{
    if (!( PINB & (1<<PB0) ))                                     // If the USI DI pin is low, then it is likely that it - mkm

/*    if (!( PINA & (1<<PA5) ))                                     // If the USI DI pin is low, then it is likely that it
*/
    {                                                             // was this pin that generated the pin change interrupt.
        TCNT0  = INTERRUPT_STARTUP_DELAY + INITIAL_TIMER0_SEED;   // Plant TIMER0 seed to match baudrate (including interrupt start up time.).

/*        TCCR0B  =(0<<CS02)|(0<<CS01)|(1<<CS00);        // Reset the pre-scaler and start Timer0.
// change mkm         TCCR0B  = (1<<PSR0)|(0<<CS02)|(0<<CS01)|(1<<CS00);
        GTCCR =  (1 << PSR10); // presummably (?) this is the same as 1<<PSR0 in the above register!! BUT the TSM bit ??? (1 << TSM) |
*/
		GTCCR = (1  << TSM ) | ( 1<< PSR0 ); // see page 114 of 24A doc also p 77 of 45 doc
//		TCCR0B = (0<<CS02)|(0<<CS01)|(1<<CS00);  			      // Reset the pre-scaler and start Timer0.
// changed from 	TCCR0B = (1<<PSR0)|(0<<CS02)|(0<<CS01)|(1<<CS00);
	TCCR0B = (0<<CS02)|(1<<CS01)|(0<<CS00);  			      // Reset the pre-scaler and start Timer0.  these bitS are for TIMER_PRESCALER 8
// to reset the prescaler, another register is used in the attiny44a
		GTCCR = (0 << TSM ); // see note on this reg above
//	GTCCR =  (1 << PSR10); // presummably (?) this is the same as 1<<PSR0 in the above register (for the 26)!!............(1 << TSM) |
        TIFR  = (1<<TOV0);                                       // Clear Timer0 OVF interrupt flag. CHANGE MKM
        TIMSK |= (1<<TOIE0);                                      // Enable Timer0 OVF interrupt.
//CHANGE MKM
        USICR  = (0<<USISIE)|(1<<USIOIE)|                         // Enable USI Counter OVF interrupt.
                 (0<<USIWM1)|(1<<USIWM0)|                         // Select Three Wire mode.
                 (0<<USICS1)|(1<<USICS0)|(0<<USICLK)|             // Select Timer0 OVER as USI Clock source.
                 (0<<USITC);
                                                                  // Note that enabling the USI will also disable the pin change interrupt.
        USISR  = 0xF0 |                                           // Clear all USI interrupt flags.
                 USI_COUNTER_SEED_RECEIVE;                        // Preload the USI counter to generate interrupt.

        GIMSK &=  ~(1<<PCIE);                                     // Disable pin change interrupt for PCINT[5:0] ... change mkm

        USI_UART_status.ongoing_Reception_Of_Package = TRUE;
    }
}

// The USI Counter Overflow interrupt is used for moving data between memory and the USI data register.
// The interrupt is used for both transmission and reception.
// #pragma vector=USI_OVF_vect
// __interrupt void USI_Counter_Overflow_ISR(void)
ISR(USI_OVF_vect) // change mkm
{
	uint8_t tmp_tx_tail;
    uint8_t tmp_rx_head;

    // Check if we are running in Transmit mode.
    if( USI_UART_status.ongoing_Transmission_From_Buffer )
    {
        // If ongoing transmission, then send second half of transmit data.
        if( USI_UART_status.ongoing_Transmission_Of_Package )
        {
            USI_UART_status.ongoing_Transmission_Of_Package = FALSE;    // Clear on-going package transmission flag.

            USISR = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);                 // Load USI Counter seed and clear all USI flags.
            USIDR = (usiuartx_tx_data << 3) | 0x07;                     // Reload the USIDR with the rest of the data and a stop-bit.
        }
        // Else start sending more data or leave transmit mode.
        else
        {
            // If there is data in the transmit buffer, then send first half of data.
            if ( usiuartx_tx_head != usiuartx_tx_tail )
            {
                USI_UART_status.ongoing_Transmission_Of_Package = TRUE; // Set on-going package transmission flag.

                tmp_tx_tail = ( usiuartx_tx_tail + 1 ) & USIUARTX_TX_BUFFER_MASK;    // Calculate buffer index.
                usiuartx_tx_tail = tmp_tx_tail;                                      // Store new index.
                usiuartx_tx_data = usiuartx_tx_buffer[tmp_tx_tail];                  // Read out the data that is to be sent. Note that the data must be bit reversed before sent.
                                                                        // The bit reversing is moved to the application section to save time within the interrupt.
                USISR  = 0xF0 | (USI_COUNTER_SEED_TRANSMIT);            // Load USI Counter seed and clear all USI flags.
                USIDR  = (usiuartx_tx_data >> 2) | 0x80;                // Copy (initial high state,) start-bit and 6 LSB of original data (6 MSB
                                                                        //  of bit of bit reversed data).
            }
            // Else enter receive mode.
            else
            {
                USI_UART_status.ongoing_Transmission_From_Buffer = FALSE;

                TCCR0B  = (0<<CS02)|(0<<CS01)|(0<<CS00);                // Stop Timer0.

                PORTB |=   (1<<PB1)|(1<<PB0);                           // Enable pull up on USI DO, DI pins. (USCK/PB2 and PB3 not needed)
                DDRB  &= ~((1<<PB1)|(1<<PB0));                          // Set USI DI, DO pins as inputs. (USCK/PB2 and PB3 not needed)

/*                PORTA |=   (1<<PA5)|(1<<PA6);                           // Enable pull up on USI DO, DI pins.
                DDRA  &= ~((1<<PA5)|(1<<PA6));                          // Set USI DI, DO pins as inputs.
// change mkm  DO is PA5 , DI is PA6
*/
                USICR  =  0;                                            // Disable USI.
                GIFR   =  (1<<PCIF);                                    // Clear pin change interrupt flag. change mkm
                GIMSK |=  (1<<PCIE);                                    // Enable pin change interrupt for PCINT[5:0] ... change mk 
                PCMSK |=  (1<<PCINT0);									// Pin Change Mask Register, enable for PCINT0/PB0
            }
        }
    }
    // Else running in receive mode.
    else
    {
        USI_UART_status.ongoing_Reception_Of_Package = FALSE;

        tmp_rx_head = ( usiuartx_rx_head + 1 ) & USIUARTX_RX_BUFFER_MASK;        // Calculate buffer index.

        if ( tmp_rx_head == usiuartx_rx_tail )                          // If buffer is full trash data and set buffer full flag.
        {
            USI_UART_status.reception_Buffer_Overflow = TRUE;           // Store status to take actions elsewhere in the application code
        }
        else                                                            // If there is space in the buffer then store the data.
        {
            usiuartx_rx_head = tmp_rx_head;                             // Store new index.
            usiuartx_rx_buffer[tmp_rx_head] = USIDR;                    // Store received data in buffer. Note that the data must be bit reversed before used.
        }                                                               // The bit reversing is moved to the application section to save time within the interrupt.

        TCCR0B  = (0<<CS02)|(0<<CS01)|(0<<CS00);                // Stop Timer0.
 
        PORTB |=   (1<<PB1)|(1<<PB0);                           // Enable pull up on USI DO, DI pins. (USCK/PB2 and PB3 not needed)
        DDRB  &= ~((1<<PB1)|(1<<PB0));                          // Set USI DI, DO pins as inputs. (USCK/PB2 and PB3 not needed)

 /*       PORTA |=   (1<<PA6)|(1<<PA5);                           // Enable pull up on USI DO, DI pins.
        DDRA  &= ~((1<<PA6)|(1<<PA5));                          // Set USI DI, DO pins as inputs.
// change mkm  DO is PA5 , DI is PA6
*/
        USICR  =  0;                                            // Disable USI.
        GIFR   =  (1<<PCIF);                                    // Clear pin change interrupt flag.
        GIMSK |=  (1<<PCIE);                                    // Enable pin change interrupt for PCINT[7:0]
        PCMSK |=  (1<<PCINT0);									// Pin Change Mask Register, enable for PCINT5/PB5 ... change mkm
    }

}

// Timer0 Overflow interrupt is used to trigger the sampling of signals on the USI ports.
// #pragma vector=TIMER0_OVF0_vect
// __interrupt void Timer0_OVF_ISR(void)
ISR(TIM0_OVF_vect) // change mkm
{
    TCNT0 += TIMER0_SEED;                   // Reload the timer,
                                            // current count is added for timing correction.
}

// ============================================================================
