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

#define FOSC    8388608
#define BAUD    4800
#define MYUBRR  FOSC/16/BAUD - 1
#define LEDOFF  PORTB &= ~(1 << PORTB7)
#define LEDON   PORTB |= (1 << PORTB7)
#define FLAG   1


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
    float relative_humidty;
    float heat_index;
    float barometric_prssr;
    float uv_index;
    int rain;
 } current_measures;

 struct AverageMeasurements 
 {
    float temperature;
    float wind_mph;
    float relative_humidty;
    float heat_index;
    float barometric_prssr;
    float uv_index;
    int rain;       // How do we want to do avg for rain? Frequencies? 
 } avg_measures;

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
// Timer 1 for polling humidity and start ADC
void init_t1()
{
   // Initialize the timer
   TCCR1A = 0;
   TCCR1B = 0;
   TCNT1  = 0;
   
   // Compare Match Register
   // 10 second delay right now
   OCR1A = 40960;
   // CTC mode
   TCCR1B |= (1 << WGM12);
   
   // Set CS12 and CS10 bits for 1024 prescaler
   TCCR1B |= (1 << CS12) | (1 << CS10);  
   
   // Enable timer compare interrupt
   TIMSK1 |= (1 << OCIE1A);
 
}

// Timer 3 for light sensors
void init_t3()
{
   // Initialize the timer
   TCCR3A = 0;
   TCCR3B = 0
   TCNT3  = 0;
   
   // Compare Match Register
   // 10 second delay right now
   OCR3A = 40960;
   // CTC mode
   TCCR3B |= (1 << WGM12);
   
   // Set CS12 and CS10 bits for 1024 prescaler
   TCCR3B |= (1 << CS12) | (1 << CS10);  
   
   // Enable timer compare interrupt
   TIMSK3 |= (1 << OCIE3A);
 
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
   TCCR4B |= (1 << WGM12);
   
   // Set CS12 and CS10 bits for 1024 prescaler
   TCCR4B |= (1 << CS12) | (1 << CS10);  
   
   // Enable timer compare interrupt
   TIMSK4 |= (1 << OCIE4A);
 
}

// Timer 1 Interrupt  ******************************* TIMER 1 **********************
int i = 0;
ISR(TIMER1_COMPA_vect)
{  
   // Toggle the LED    
   PINB |= (1<<PINB7);
//   uart_write_uint16(i);
//   uart_putstring("\n");
//   i++;
//   if (i == 65535)
//   {
//     i = 0;
//   }

   // Poll I2C Sensors
   current_measures.barometric_prssr = poll_pressure();
   current_measures.relative_humidity = poll_humidity();
   
   // ADC Sensors
   // If this doesnt work, then individual functions that change MUX value in ADMUX
   current_measures.temperature = adc_get_conversion(TEMP_CH);
   current_measures.rain = adc_get_conversion(RAIN_CH);
   current_measures.wind_mph = adc_get_conversion(WIND_CH);
   //current_measures.uv_index = adc_get_conversion(UV_CH);

   // Calculate Relative Humidity with measured humidity and temperature
   current_measures.heat_index = calc_hi();

   // Check the values against thresholds
   check_thresholds();
}

// Timer 3 Interrupt      *************************** TIMER 3 *********************
int TIMER3_COUNT = 0;
ISR(TIMER3_COMPA_vect)
{
  // Every 20 Minutes
  if (TIMER3_COUNT == 120)
  {
    current_measures.uv_index =  get_adc_conversion(UV_CH);
    // Stretch Goals: Check Lux levels on other light diode 
  }
  else
  {
    TIMER3_COUNT++;
  }
}

// Timer 4 Interrupt      *************************** TIMER 4 *********************
int TIMER4_COUNT = 0;
ISR(TIMER4_COMPA_vect)
{
  // Every 5 Minutes
  if (TIMER4_COUNT == 30)
  {
    // Poll BP
    current_measures.barometric_prssr = poll_bp();
    TIMER4_COUNT = 0;
  }
  else if
  {
    TIMER4_COUNT++;
  }
  
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
 *                Sensor Functions
 *********************************************/
void init_bp(int flag)
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

float poll_humidity()
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

/*********************************************
 *             Aditional Functions
 *********************************************/
void check_tresholds()                  ****************** threshold checks ******************
{
  // 0 - Closed     1 - Open
  int roof_state = get_roof_state();

  if (current_measures.temperature > TEMP_MAX_THRESHOLD)
  {
    uart_put_string("Temperature max triggered");
    if (roof_state != TEMP_MAX)
      change_roof_state(TEMP_MAX)

  }
  else if (current_measures.temperature < TEMP_MIN_THRESHOLD)
  {
    uart_put_string("Temperature min triggered");
    if (roof_state != TEMP_MIN)
      change_roof_state(TEMP_MIN)
  }
  if (current_measures.wind_mph > WIND_THRESHOLD)
  {
    uart_put_string("Wind triggered");
    if (roof_state != WINDY)
      change_roof_state(WINDY)
  }
  if (current_measures.relative_humidity > RH_THRESHOLD)
  {
    uart_put_string("Humidity triggered");
    if (roof_state != RH)
      change_roof_state(RH)
  }
  if (current_measures.heat_index > HI_THRESHOLD)
  {
     uart_put_string("Heat index triggered");
     if (roof_state != HI)
      change_roof_state(HI)
  }
  if (current_measures.barometric_prssr > BP_THRESHOLD)
  {
    uart_put_string("Barometric Pressure triggered");
    if (roof_state != BP)
      change_roof_state(BP)
  }
  if (current_measures.uv_index > UV_THRESHOLD)
  {
    uart_put_string("UV Index triggered");
    if (roof_state != UV)
      change_roof_state(UV)
  }
  if (current_measures.rain)
  {
    uart_put_string("Rain triggered");    
    if (roof_state != RAIN)
      change_roof_state(RAIN)
  }
}

int get_roof_state()
{
  // Now this is going to be either check proximity sensors or just check global? 
  return poll_proximity_sensors();
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
                     (0.05481717 * t_rh * t_rh) + (0.0012287 t_temp * t_temp *t_rh) +
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
  
  // LED on Bluno Mega is PB7
  DDRB |= (1 <<DDB7); 

  cli();//stop interrupts

  // Initialize the Timers
  init_t1();          // Timer 1 for polling sensors
  init_t3();          // Timer 3 for light polling
  init_t4();          // Timer 4 for polling barometric pressure
  
  sei();//allow interrupts

  // Initialize the I2C
  init_twi();
  
  // Initialize the USART 
  // Baud is 9600 in serial monitor 
  uint8_t ubrr = MYUBRR;
  init_uart0(ubrr);

  // Initialize the ADC
  init_adc(); 

  uart_putstring("start adc\n");
  // Get ADC reading on channel 0
  //uint16_t adc_result = adc_get_conversion(0);
  //adc_result >> 2;
  //uart_write_uint16(adc_result + '0');
}

void loop() 
{
  // If the chip receives anything, transmit it back
  if (UCSR0A & (1 << RXC0))
  {
    uart_tx(uart_rx());
  }
}
