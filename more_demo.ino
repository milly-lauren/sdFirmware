/*********************************************
 *           Group 8 Senior Desgin           
 * File: demo                                  
 * Author: Lauren Miller                     
 * Description: Software for midterm demo    
 * Tests PWM and timers                                               
 * 
 * link for schematic and pins: https://wiki.dfrobot.com/Bluno_Mega_1280__SKU_DFR0306_
 * 
 * INSTRUCTIONS
 * Connect Actuators to PWMs 
 * Dev Board PWML pin 5 and PWMH pin 6
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

/********************************
 *      Sensor Measurements
 ********************************/
 struct CurrentMeasurements 
 {
    float temperature;
    float wind_mph;
    float relative_humidity;
    float heat_index;
    float barometric_prssr;
    float uv_index;
    int rain;
 } current_measures;

 struct AverageMeasurements 
 {
    float temperature;
    float wind_mph;
    float relative_humidity;
    float heat_index;
    float barometric_prssr;
    float uv_index;
    int rain;       // How do we want to do avg for rain? Frequencies? 
 } avg_measures;

/********************************
 *          Wind Globals
 ********************************/
#define VaneOffset 0; // define the anemometer offset from magnetic north
int VaneValue; // raw analog value from wind vane
int Direction; // translated 0 - 360 direction
int CalDirection; // converted value with offset applied
int LastValue; // last direction value

int SampleRequired; // this is set true every 3s. Get wind speed
volatile unsigned long Rotations; // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in isr


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

// Timer 5 for wind sensor debouncing
void init_t5()
{
   // Initialize the timer
   TCCR5A = 0;
   TCCR5B = 0;
   TCNT5  = 0;
   
   // Compare Match Register
   // 3 second delay
   OCR5A = 12288;
   // CTC mode
   TCCR5B |= (1 << WGM52);
   
   // Set CS12 and CS10 bits for 1024 prescaler
   TCCR5B |= (1 << CS52) | (1 << CS50);  
   
   // Enable timer compare interrupt
   TIMSK5 |= (1 << OCIE5A);
 
}

// Timer 5 Interrupt (Wind)
ISR(TIMER5_COMPA_vect)
{
  SampleRequired = 1;
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
void init_adc()
{

 /* Enable clock to write ADC registers */
  PRR0 &= ~(1 << PRADC);

  ADMUX = (1 << REFS1)   /* Internal 1.1 V*/
          | (0 << ADLAR)    /* Left Adjust Result: disabled */
          | (0x00 << MUX0); /* ADC Single Ended Input pin 0 */

  ADCSRA = (1 << ADEN)        /* ADC: enabled */
           | (0 << ADATE)     /* Auto Trigger: disabled */
           | (0 << ADIE)      /* ADC Interrupt: disabled */
           | (1 << ADPS1) | (1 << ADPS2); /* /64 for 128kHz*/
  ADCSRB = (0x00 << ADTS0)    /* Free Running mode */
           | (0 << ACME);      /* Analog Comparator Multiplexer: disabled */

  // Perform first initlization (takes 25 ADC clock cycles)
  ADCSRA |= (1 << ADSC);
}

uint8_t adc_get_conversion(uint8_t channel)
{
  uint8_t result;
  adc_start_conversion(channel);
  while (!adc_done());
  result = adc_result();
  ADCSRA |= (1 << ADIF);
  return result;
}

void adc_start_conversion(uint8_t channel)
{
  ADMUX &= ~0x1f;
  ADMUX |= channel;
  ADCSRA |= (1 << ADSC);      // Starts Conversion
}

bool adc_done()
{
  return ((ADCSRA & (1 << ADIF)));
}

uint16_t adc_result()
{
  return (ADCL | ADCH << 8);
}


/*********************************************
*                PWM Functions
*********************************************/

void PWM0_disable()
{
 // Return to non-operating mode on OC0A pin
  TCCR0A &= ~((0 << COM0A1) | (0 << COM0A0));

  //Return to non-operating mode on OC0B. pin
  TCCR0B &= ~((0 << COM0B1) | (0 << COM0B0));
}

void PWM0_enable()
{
  // Clear OC0A on Compare Match, set OC0A  at BOTTOM
  // Non-inverting mode
  TCCR0A |= ((1 << COM0A1) | (0 << COM0A0));
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
void init_wind()
{
  SampleRequired = false;
  Rotations = 0;
  // Set Wind Pin (A0 pin 78) to input and enable external interrupts for falling edge
  DDRE |= (0 << WIND_PIN);
  EICRB |= (1 << ISC41) | (1 << ISC40);         // External interrupt (Wind) - rising edge between two sample results in interrupt
  EIMSK |= (1 << INT4);                         // External interrupt INT4 Pin 6
  EIFR |= (0 << INTF4);                         // Clear ext int flag
  
}

void poll_wind()       
{
  if (SampleRequired)
  {
    // Velocity = P(2.25/T) = Roations * (2.25/3s) = R * 0.75
    current_measures.wind_mph = Rotations * 0.75;
    Rotations = 0;
    uart_putstring("Wind Speed: ");
    uart_write_uint16(current_measures.wind_mph);

    SampleRequired = 0;
  }
}

// This is the function that the interrupt calls to increment the rotation count
ISR(INT4_vector) 
{
  if((millis() - ContactBounceTime) > 15 ) 
  {   
    // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }
}


/*********************************************
 *             Aditional Functions
 *********************************************/
void check_thresholds()                 // ****************** threshold checks ******************
{
  // 0 - Closed     1 - Open
  int roof_state = get_roof_state();

  if (current_measures.wind_mph > WIND_THRESHOLD)
  {
    // uart_putstring() in place of bluetooth notficiation
    uart_putstring("Wind triggered");
    if (roof_state != WINDY)
      change_roof_state(WINDY);
  }
}

int get_roof_state()
{
  // Now this is going to be either check proximity sensors or just check global? 
  return ROOF_STATUS;
  //return poll_proximity_sensors();
}

void change_roof_state(int new_setting)
{
  PWM0_enable();

  
  // Set delay here or put in proximity 
  delay(3000);
  // int cur_roof = get_roof_status();      // this would be polling the proximity sensor s 
  // while(get_roof_status() != new_setting)
  //{
  //  
  //}
  PWM0_disable();

  //Change the roof status global 
  ROOF_STATUS = new_setting;
}


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
  init_t0();          // Timer 0 for PWMs (PWM init)
  init_t1();          // Timer 1 for polling sensors
  init_t5();          // Timer 5 for debouncing wind sensor

  // Initialize the I2C
  init_twi();
  
  // Initialize the USART 
  // Baud is 9600 in serial monitor 
  uint8_t ubrr = MYUBRR;        // Check macros for definitiona
  init_uart0(ubrr);

  // Initialize the ADC
  // Starts first conversion which will take 25 clock cycles at 128kHz
  init_adc(); 

  // Initialize Sensors
  // Initialize Wind
  init_wind();


  // Set Global Inerrupt Enable
  sei();

}

void loop() 
{
  // put your main code here, to run repeatedly:

}
