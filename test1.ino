/*********************************************
 *           Group 8 Senior Desgin           *
 * File: test1                               *  
 * Author: Lauren Miller                     *
 * Description: v1 of firmware for Bluno     *
 *  Mega development board for initializing  *
 *  hardware and communication with sensors. *
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

volatile unsigned int TIMER1_COUNT;
volatile unsigned int TIMER4_COUNT;


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

// Timer 3 - 16 bit Open
void init_t3()
{
   // Initialize the timer
   TCCR3A = 0;
   TCCR3B = 0;
   TCNT3  = 0;
 
}

// Timer 4 for polling Barometric Pressure
void init_t4()
{
   // Initialize the timer
   TCCR4A = 0;
   TCCR4B = 0;
   TCNT4  = 0;
   
   // Compare Match Register
   // 10 second delay right now
   OCR4A = 40960;
   // CTC mode
   TCCR4B |= (1 << WGM42);
   
   // Set CS12 and CS10 bits for 1024 prescaler
   TCCR4B |= (1 << CS42) | (1 << CS40);  
   
   // Enable timer compare interrupt
   TIMSK4 |= (1 << OCIE4A);
 
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

// Timer 1 Interrupt  ******************************* TIMER 1A ISR **********************
ISR(TIMER1_COMPA_vect)
{  
   
   TIMER1_COUNT++;
   uart_write_uint16(TIMER1_COUNT);
   uart_putstring("\n");
   if (TIMER1_COUNT >= 4)
   {
     // Toggle the LED 
     uart_putstring("toggle LED");   
     PINB |= (1<<PINB7);
     TIMER1_COUNT = 0;
   }
   
//   // Poll I2C Sensors
//   current_measures.barometric_prssr = poll_bp();
//   current_measures.relative_humidity = poll_humidity();
//   
//   // ADC Sensors
//   // If this doesnt work, then individual functions that change MUX value in ADMUX
//   current_measures.temperature = adc_get_conversion(TEMP_CH);
//   current_measures.rain = adc_get_conversion(RAIN_CH);
//   //current_measures.uv_index = adc_get_conversion(UV_CH);
//
//   // Calculate Relative Humidity with measured humidity and temperature
//   current_measures.heat_index = calc_hi();
//
//   // Check the values against thresholds
//   check_thresholds();
}

// Timer 1B Interrupt      *************************** TIMER 1B ISR *********************
int TIMER1B_COUNT = 0;
ISR(TIMER1_COMPB_vect)
{
  // Every 20 Minutes
  if (TIMER1B_COUNT == 120)
  {
    current_measures.uv_index =  adc_get_conversion(UV_CH);
    // Stretch Goals: Check Lux levels on other light diode 
    TIMER1B_COUNT = 0;
  }
  else
  {
    TIMER1B_COUNT++;
  }
}

// Timer 3 Interrupt      *************************** TIMER 3 ISR *********************
ISR(TIMER3_COMPA_vect)
{
  
}

// Timer 4 Interrupt      *************************** TIMER 4 ISR *********************
ISR(TIMER4_COMPA_vect)
{
   
//  // Every 5 Minutes
//  if (TIMER4_COUNT == 30)
//  {
//    // Poll BP
//    current_measures.barometric_prssr = poll_bp();
//    TIMER4_COUNT = 0;
//  }
//  else
//  {
//    TIMER4_COUNT++;
//  }
  
}

// Timer 5 Interrupt      *************************** TIMER 5 ISR *********************
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

void poll_wind()       //  ****************************** Wind Functions********************
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

void init_bp()        //  ****************************** BP Functions********************
{
    // Function used to initialize the pressure sensor
    // Set mode to forced, every time set to forced = measurement
    // THIS CODE IS ASSUMING SDO IS CONNECTED TO V (NOT GROUND)
    
    // Slave address
    uint8_t addr = 0xEE;     
            
    twi_start();
    twi_write(addr);
    if (twi_status() != 0x18)       // Need to check this value for ACK
    {
        return;
    }
    
    // First initiation, writing mode = 01 triggers a measurement
    uint8_t reg = 0xF4;
    uint8_t ctrl = 0x05;
    twi_write(reg);
    twi_write(ctrl);
    twi_stop();
   
    return;
}

float poll_bp()
{
    // Barometric Pressure Address
    // 0x76 or 0x77 based on SDO pin
    uint8_t addr = 0xEE;
    uint8_t p_LSB = 0xF7;
    
    // Send address and START command
    twi_start();
    twi_write(addr);
    twi_write(p_LSB);
    
    // Send second START command
    // NOW address is in write mode
    addr = 0xEF;
    twi_start();
    twi_write(addr);
    
    // Read the data for pressure and stop 
    uint8_t p_r_MSB = twi_read_ack();
    uint8_t p_r_LSB = twi_read_nack();
    twi_stop();

    
    // Equations
    float pressure = 1;
    
    return pressure;
}

float poll_humidity()       //  ****************************** Humidity Functions********************
{
    // Humidity Sensor Address(s)
    uint8_t addr_w = 0x70;
    uint8_t cmdMSB = 0x5C;
    uint8_t cmdLSB = 0x24;
    // * clocking stretching command = 0x58E0
    
    // START then 0x70 then command for H first (MSB first)
    twi_start();
    
    // Wait for measurement (Can check current?)
    
    // Send START again and read command 
    uint8_t addr_r = 0x71;
    
    // Read H (MSB first, LSB, CRC)
    
    // Send NACK to not get temperature reading 
    
    float humidity = 1;
    return humidity;
}


void init_prox()                      // ****************** proximity sensor ******************
{
  
}


void poll_prox()
{

}



/*********************************************
 *             Aditional Functions
 *********************************************/
void check_thresholds()                 // ****************** threshold checks ******************
{
  // 0 - Closed     1 - Open
  int roof_state = get_roof_state();

  if (current_measures.temperature > TEMP_MAX_THRESHOLD)
  {
    uart_putstring("Temperature max triggered");
    if (roof_state != TEMP_MAX)
      change_roof_state(TEMP_MAX);
  }
  else if (current_measures.temperature < TEMP_MIN_THRESHOLD)
  {
    uart_putstring("Temperature min triggered");
    if (roof_state != TEMP_MIN)
      change_roof_state(TEMP_MIN);
  }
  if (current_measures.wind_mph > WIND_THRESHOLD)
  {
    uart_putstring("Wind triggered");
    if (roof_state != WINDY)
      change_roof_state(WINDY);
  }
  if (current_measures.relative_humidity > RH_THRESHOLD)
  {
    uart_putstring("Humidity triggered");
    if (roof_state != RH)
      change_roof_state(RH);
  }
  if (current_measures.heat_index > HI_THRESHOLD)
  {
     uart_putstring("Heat index triggered");
     if (roof_state != HI)
      change_roof_state(HI);
  }
  if (current_measures.barometric_prssr > BP_THRESHOLD)
  {
    uart_putstring("Barometric Pressure triggered");
    if (roof_state != BP)
      change_roof_state(BP);
  }
  if (current_measures.uv_index > UV_THRESHOLD)
  {
    uart_putstring("UV Index triggered");
    if (roof_state != UV)
      change_roof_state(UV);
  }
  if (current_measures.rain)
  {
    uart_putstring("Rain triggered");    
    if (roof_state != RAIN)
      change_roof_state(RAIN);
  }
}

int get_roof_state()
{
  // Now this is going to be either check proximity sensors or just check global? 
  
  return poll_proximity_sensors();
}

void change_roof_state(int new_setting)
{
  PWM0_enable();

  
  // Set delay here or put in proximity 
  /*
   * int cur_roof = get_roof_status();      // this would be polling the proximity sensor s 
   * 
   */

   PWM0_disable();
  
}

int poll_proximity_sensors()
{
  // Figure if roof is open or closed
  return 1;
}

// Calculat Heat Index based on temperature and humidity measurements
float calc_hi()
{
  float t_temp = current_measures.temperature;  
  float t_rh = current_measures.relative_humidity;      // *** Need to be integer % 

  float heat_index = (-42.379) + 2.04901523 * t_temp + 10.1433127 * t_rh -
                     (0.22475541 * t_temp * t_rh) - (0.00683783 * t_temp * t_temp) -
                     (0.05481717 * t_rh * t_rh) + (0.0012287 * t_temp * t_temp * t_rh) +
                     ( 0.00085282 * t_temp * t_rh * t_rh ) - 
                     (0.00000199 * t_temp * t_temp * t_rh * t_rh);

   return heat_index;              
}



/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/
/*******************************************************************************/

// Main Functions
void setup() 
{
  // Set up code
  
  // LED on Bluno Mega is PB7 (Will be also set for the actuator PWMs)
  DDRB |= (1 <<DDB7); 

  // Set pins for PWM
  DDRG |= (1 << DDG5);
  
  // Disable Interrupts
  cli();

  // Initialize the Timers
  init_t0();          // Timer 0 for PWMs (PWM init)
  init_t1();          // Timer 1 for polling sensors
  init_t3();          // Timer 3 open (can do PWMs with 16 bit)
  init_t4();          // Timer 4 for polling barometric pressure 
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
  
   // Initilaize Barometrice Pressure
  init_bp();


  // Set Global Inerrupt Enable
  sei();

  // Get ADC reading on channel 0
  //uint16_t adc_result = adc_get_conversion(0);
  //adc_result >> 2;
  //uart_write_uint16(adc_result + '0');
}

void loop() 
{
  // If the chip receives anything, transmit it back
//  if (UCSR0A & (1 << RXC0))
//  {
//    uart_tx(uart_rx());
//  }
}
