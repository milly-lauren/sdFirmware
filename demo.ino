/*********************************************
 *           Group 8 Senior Desgin           
 * File: demo                                  
 * Author: Lauren Miller                     
 * Description: Software for midterm demo    
 * Tests PWM and ADC                                                 
 * 
 * link for schematic and pins: https://wiki.dfrobot.com/Bluno_Mega_1280__SKU_DFR0306_
 * 
 * INSTRUCTIONS
 * Connect Actuators to PWMs 
 * Dev Board PWML pin 5 and PWMH pin 6
 * (PG5 & PB7)
 * 
 * Start with roof OPEN
 * 
 * Connect annenometer to pin PWML pin 3 
 * (Should be INT4 (For external interrupt) PE4 on 1280)
 * 
 *********************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#define FOSC        8388608
#define BAUD        4800
#define MYUBRR      FOSC/16/BAUD - 1
#define LEDOFF      PORTB &= ~(1 << PORTB7)
#define LEDON       PORTB |= (1 << PORTB7)
#define FLAG        1
#define WIND_PIN    DDE4

/*********************************************
 *              Roof Characterisitcs
 *********************************************/

/********************************
 *     Sensor Thresholds
 ********************************/
#define TEMP_MAX_THRESHOLD    100
#define TEMP_MIN_THRESHOLD    32
#define WIND_THRESHOLD        28
#define RH_THRESHOLD          82
#define HI_THRESHOLD          110
#define BP_THRESHOLD          101.325
#define UV_THRESHOLD          11

/********************************
 *        Roof Algorithms
 *        0 - Close
 *        1 - Open
 ********************************/
#define WINDY                 0
#define RAIN                  0
#define TEMP_MAX              1
#define TEMP_MIN              0
#define RH                    0
#define HI                    0
#define BP                    0
#define UV                    0
// Roof Status global 
volatile int ROOF_STATUS;

/*****************************************************************************************/
/*****************************************************************************************/
/*****************************************************************************************/

/*********************************************
 *              Hardware Functions
 *********************************************/

/********************************
 *       Hardware Constants
 ********************************/
#define TEMP_CH               0x00
#define RAIN_CH               0x01
#define WIND_CH               0x02
#define UV_CH                 0x03
#define LUX_CH                0x04

/********************************
 *        Timer Functions
 ********************************/
// Timer 0 (A&B) for PWM (Also init_pwm())
void init_t0()
{
  /* Enable TC0 */
  PRR0 &= ~(1 << PRTIM0);
  TCCR0A = 0;
  TCCR0B = 0;

  TCCR0A = (0 << COM0A1) | (0 << COM0A0)   /* Normal port operation, OCA disconnected */
           | (0 << COM0B1) | (0 << COM0B0) /* Normal port operation, OCB disconnected */
           | (1 << CS00)                   /* No prescalar */
           | (1 << WGM01) | (1 << WGM00);  /* Mode 3 - Fast PWM; TOP : 0xFF, Update OCRx at TOP, set TOV flag at MAX */

  TCCR0B = (0 << CS02) | (1 << WGM02);    /* No prescalar and Fast PWM mode */

}
 
// Timer 1 for polling humidity and start ADC
void init_t1()
{
   // Initialize the timer
   TCCR1A = 0;
   TCCR1B = 0;
   TCNT1  = 0;
   
   // Compare Match Register
   // 5 second for regular sensors
   OCR1A = 40960;
   // 15 second for light sensors
   //OCR1B = 61439;
   // CTC mode
   //TCCR1B |= (1 << WGM12);
   // Normal 
   TCCR1B = 0;
   // Set CS12 and CS10 bits for 1024 prescaler
   TCCR1B |= (1 << CS12) | (1 << CS10);  
   
   // Enable timer compare interrupt
   TIMSK1 |= (1 << OCIE1A);
   // TIMSK1 |= (1< << OCIE1B);
 
}

/********************************
 *        UART Functions
 ********************************/
void init_uart0(uint8_t ubrr)
{
    // Set baud rate
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    // Set frame format to 8 data bits, no parity, 1 stop bit
    // UCSR0C |= /*(1<<USBS0)|*/(3<<UCSZ00);
    UCSR0C = (1<<UCSZ00)| (1<<UCSZ01);
    // Enable transmission and reception, disable the empty buffer interrupt, option: RXCEI0 will interrupt when UART received
    UCSR0B |= (1<<RXEN0)|(1<<TXEN0);

}

void uart_tx(unsigned char data)
{
    while(!(UCSR0A & (1 << UDRE0)))
    {
        
    }
    UDR0 = data;
}

unsigned char uart_rx()
{
    while(!(UCSR0A & (1 << RXC0)))          // False when have no RX data
    {
        
    }
    return UDR0;
}

void uart_putstring(char * strptr)
{
  while(*strptr != 0x00)
  {
    uart_tx(*strptr);
    strptr++;
  }
}

void uart_write_uint16(uint16_t n)
{
   int digit;
   if(n >= 10000)
   {
     digit = (n/10000) % 10;
     uart_tx(digit + '0');
   }
   if(n >= 1000)
   {
     digit = (n/1000) % 10;
     uart_tx(digit + '0');
   }
   if(n >= 100)
   {
     digit = (n/100) % 10;
     uart_tx(digit + '0');
   }
   if(n >= 10)
   {
     digit = (n/10) % 10;
     uart_tx(digit + '0');
   }
   digit = n % 10;
   uart_tx(digit + '0');
}


/*********************************************
*                PWM Functions
*********************************************/

void PWM0_disable()
{
 // Return to non-operating mode on OC0A pin
  TCCR0A &= ~((0 << COM0A1) | (0 << COM0A0));

  //Return to non-operating mode on OC0B pin
  TCCR0B &= ~((0 << COM0B1) | (0 << COM0B0));
}

void PWM0_enable()
{
  init_t0();          // Timer 0 for PWMs (PWM init)
  // Clear OC0A on Compare Match, set OC0A  at BOTTOM
  // Non-inverting mode
  TCCR0A |= ((1 << COM0A1) | (0 << COM0A0));
  TCNT0 = 0;
  OCR0A = 255;

   // Clear OC0B on Compare Match, set OC0B at BOTTOM
  // Non-inverting mode
  TCCR0B |= ((1 << COM0B1) | (0 << COM0B0));
  OCR0B = 254;
}




/*********************************************
 *                Sensor Functions
 *********************************************/
 
/****************************** Wind Functions********************/


/*****************************************************************************************/
/*****************************************************************************************/
/*****************************************************************************************/


void setup() 
{
  //Start with roof OPEN
  ROOF_STATUS = 1;
  
  // Set pins for PWM
  DDRG |= (1 << DDG5);
  DDRB |= (1 << DDB7);
  
  // Disable Interrupts
  cli();

  // Initialize the Timers
  init_t1();          // Timer 1 for polling sensors
  // Initialize the USART 
  // Baud is 9600 in serial monitor 
  uint8_t ubrr = MYUBRR;        // Check macros for definitiona
  init_uart0(ubrr);


  // Set Global Inerrupt Enable
  sei();

}

void loop() 
{
  // If the chip receives anything, transmit it back
  if (UCSR0A & (1 << RXC0))
  {
    if (!strcmp(uart_rx(), "start")) 
    {
      // Move roof
      PWM0_enable();
    }
    else if(!strcmp(uart_rx(), "start"))
    {
      PWM0_disable();
    }
  }

}
