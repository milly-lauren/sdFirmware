#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define FOSC    8388608
#define BAUD    4800
#define MYUBRR  FOSC/16/BAUD - 1
#define LEDOFF  PORTB &= ~(1 << PORTB7)
#define LEDON   PORTB |= (1 << PORTB7)

/********************************
 *        Timer Functions
 ********************************/
void init_t1()
{
   // Initialize the timer
   TCCR1A = 0;// set entire TCCR1A register to 0
   TCCR1B = 0;// same for TCCR1B
   TCNT1  = 0;//initialize counter value to 0
   
   // Compare Match Register
   OCR1A = 10240;
   // CTC mode
   TCCR1B |= (1 << WGM12);
   
   // Set CS12 and CS10 bits for 1024 prescaler
   TCCR1B |= (1 << CS12) | (1 << CS10);  
   
   // Enable timer compare interrupt
   TIMSK1 |= (1 << OCIE1A);
 
}

ISR(TIMER1_COMPA_vect)
{ 
      //uart_tx("here"); 
      PINB |= (1<<PINB7);
        
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
    UCSR0C |= (1<<USBS0)|(3<<UCSZ00);
    // Enable transmission and reception, disable the empty buffer interrupt, option: RXCEI0 will interrupt when UART received
    UCSR0B |= (1<<RXEN0)|(1<<TXEN0);

}

void uart_tx(const uint8_t u8Data)
{
    while(!(UCSR0A & (1 << UDRE0)))
    {
        
    }
    UDR0 = u8Data;
}

uint8_t uart_rx()
{
    while(!(UCSR0A & (1 << RXC0)))          // False when have not RX data
    {
        
    }
    return UDR0;
}

/********************************
 *        I2C Functions
 ********************************/
void init_twi()
{
    // Set SCL (THESE VALUES NEED TO BE CHECKED)
    TWSR = 0x00;
    TWBR = 0x20;
    
    // Enable TWI
    TWCR = (1<<TWEN) | (1<<TWIE) | (0 <<TWEA);
    
    return;
}

void twi_start()
{
    // Send start signal
    TWCR = (1 << TWINT) | (1<<TWSTA) | (1<<TWEN);
    while((TWCR & (1<<TWINT)) == 0);
    return;
}

void twi_stop()
{
    // Send stop signal
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 <<TWEN);
    return;
}

void twi_write(uint8_t data)
{
    // Set the data to be transmitted and wait for completion
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while((TWCR && (1<<TWINT)) == 0);
    return;
}

uint8_t twi_read_ack()
{
    // Read byte with ACK
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while((TWCR & (1<<TWINT)) == 0);
    return TWDR;
}

uint8_t twi_read_nack()
{
    // Read byte with NACK
    TWCR = (1 << TWINT) | (1 << TWEN);
    TWCR &= ~(1 << TWEA);                   // Set up NACK
    while((TWCR & (1<<TWINT)) == 0);
    return TWDR;
}

uint8_t twi_status()
{
    // Get the status of TWI
    uint8_t status;
    status = TWSR & 0xF8;
    return status;
}

/********************************
 *        ADC Functions
 ********************************/

void ADC_0_init()
{

 /* Enable clock to write ADC registers */
  PRR0 &= ~(1 << PRADC);

  ADMUX = (0x00 << REFS0)   /* AREF, Internal Vref turned off */
          | (0 << ADLAR)    /* Left Adjust Result: disabled */
          | (0x00 << MUX0); /* ADC Single Ended Input pin 0 */

  ADCSRA = (1 << ADEN)        /* ADC: enabled */
           | (0 << ADATE)     /* Auto Trigger: disabled */
           | (0 << ADIE)      /* ADC Interrupt: disabled */
           | (0x01 << ADPS0); /* 2 */
  ADCSRB = (0x00 << ADTS0)    /* Free Running mode */
           | (0 << ACME);      /* Analog Comparator Multiplexer: disabled */

}

void ADC_0_start_conversion(uint8_t channel)
{
  ADMUX &= ~0x1f;
  ADMUX |= channel;
  ADCSRA |= (1 << ADSC);
}

bool ADC_0_done()
{
  return ((ADCSRA & (1 << ADIF)));
}

uint16_t ADC_0_result()
{
  return (ADCL | ADCH << 8);
}

/*******************************************************************************/

// Main Functions
void setup() 
{
  // Set up code
  
  // LED on Bluno Mega is PB7
  DDRB |= (1 <<DDB7); 

  cli();//stop interrupts

  // Initialize the Timer
  init_t1();
  
  sei();//allow interrupts

  // Initialize the I2C
  init_twi();
  
  // Initialize the USART 
  // Baud is 9600 in serial monitor 
  uint8_t ubrr = MYUBRR;
  init_uart0(ubrr);

  // ADC is Port F
}

void loop() 
{
  uart_tx(uart_rx());
}
