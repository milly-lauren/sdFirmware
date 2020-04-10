/*
 * Final version of SD Firmware Software
 * Author: Lauren Miller
 * Created:4/10/2020 
 * 
 * Sensors included:
 *  - Temperature
 *  - Rain
 *  - Humidity
 *  - UV
 *  
 * To Do:
 *  -Finish UV light functions
 *  -Send average data to the phone, 
 *    should just be opposite as read
 *    
 *   
 *    
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

/********************************
 *      Clock Constants
 *******************************/
#define FOSC        8388608
// #define FOSC     16777216
// #define BAUD     9600
#define BAUD        4800
#define MYUBRR      FOSC/16/BAUD - 1

/********************************
 *       Hardware Constants
 ********************************/
#define TEMP_CH               0
#define RAIN_CH               1
#define UV_CH                 2
#define ACTUATOR_PIN          4
#define TEMP_BETA          3988
#define R_BALANCE         10000     // 10 kOhms
#define ROOM_TEMP        298.15     // in Kelvins

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

/********************************
 *      Threshod/Behaviors
 ********************************/
 struct ThresholdMeasurements 
 {
    float max_temp;
    float min_temp;
    float wind_mph;
    float relative_humidity;
    float heat_index;
    float uv_index;
 } sensor_thresholds;

 struct RoofBehaviors 
 {
    int max_temp;
    int min_temp;
    int wind_mph;
    int relative_humidity;
    int heat_index;
    int uv_index;
    int rain;       
 } roof_behaviors;

/********************************
 *      Sensor Measurements
 ********************************/
 struct CurrentMeasurements 
 {
    float temperature;
    float wind_mph;
    float relative_humidity;
    float heat_index;
    float uv_index;
    int rain;
 } current_measures;

 struct AverageMeasurements 
 {
    float temperature;
    float wind_mph;
    float relative_humidity;
    float heat_index;
    float uv_index;
    int rain;       // How do we want to do avg for rain? Frequencies? 
 } avg_measures;

/********************************
 *           Globals
 ********************************/
int TIMER0_COUNT;
int TIMER1_COUNT;
int MEASUREMENTS_COUNT;               // Used for averaging sensor measurements
// START WITH ROOF OPEN = 1
int ROOF_STATUS;                  // 1 = open 0 = close (in place of proximity sensor)

/********************************
 *        Timer Functions
 ********************************/
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

void init_t1()
{
   // Initialize the timer
   TCCR1A = 0;
   TCCR1B = 0;
   TCNT1  = 0;
   
   // Compare Match Register (4096 = 4.2 sec)
   OCR1A = 49152;
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

ISR (TIMER1_COMPA_vect)
{
  // The current timer interrupts every 4 seconds, so when timer count is 28 poll
  TIMER1_COUNT++;
  if (TIMER1_COUNT >= 7)
  {
   // Poll the different sensors   
   PINB |= (1<<PINB7);
   TIMER1_COUNT = 0;
   poll_sensors();
  }
}


/********************************
 *        Uart Functions
 ********************************/
void init_uart0(uint8_t ubrr)
{
    // Set baud rate
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)ubrr;
    // Set frame format to 8 data bits, no parity, 1 stop bit
    // UCSR0C |= /*(1<<USBS0)|*/(3<<UCSZ00);
    UCSR0C = (1<<UCSZ00)| (1<<UCSZ01);
    // Enable transmission and reception, disable the empty buffer interrupt
    UCSR0B |= (1<<RXEN0)|(1<<TXEN0);

    // Enable the UART Interrupts
    UCSR0B |= (1 << RXCIE0);   

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

// UART Receive interrupt - for when Bluetooth sends a command
ISR(USART0_RX_vect)
{
    // Disable UART RX so the current one is not interrupted
    UCSR0B &= ~(1 << RXCIE0);  

    // Read command from buffer
    uint8_t cmd = UDR0;

    if (cmd == 0x53)
    {
      // S = change status of the roof
      // Get the current roof status and swich it
      // If the current roof is open (1) then close it (0) and vice versa
      int new_roof = get_roof_state() ? 0 : 1;
      change_roof_state(new_roof);      
    }
    else if (cmd == 0x57)
    { 
      // W = new wind threshold
      // Read the new threshold and save to the structure
      read_new_threshold(2);
    }
    else if (cmd == 0x55)
    {
      // U = new UV threshold 
      // Read the new threshold and save to the structure
      read_new_threshold(3);
    }
    else if (cmd == 0x4C)
    {
      // L = new lower temperature threshold
      // Read the new threshold value and save to the structure
      read_new_threshold(4);
    }
    else if (cmd == 0x48)
    {
      // L = new lower temperature threshold
      // Read the new threshold value and save to the structure
      read_new_threshold(5);
    }
    else if (cmd == 0x52)
    {
      // R = new rain threshold 
      // Just change open/close preference
      uint8_t behavior = uart_rx();

      if(behavior == "C" | behavior == 0x43)
      {
        // Close
        roof_behaviors.rain = 0;
      }
      else 
      {
        // Close
        roof_behaviors.rain = 1;
      }
    }

    // Enable the UART Interrupts again
    UCSR0B |= (1 << RXCIE0);  
}

/********************************
 *      Bluetooth Functions
 ********************************/
// This is the function to call when a new  
void loop_bluetooth()
{
  
}
 
void send_bluetooth(float measurement)
{
  uint8_t whole = measurement;
  uint8_t dec = (measurement - whole) * 100;
}

/********************************
 *        Sensor Functions
 ********************************/
// Read any new threshold values
void read_new_threshold( int sensor)
{
  // Sensor specifies which sensor the threshold is for
  uint8_t threshold_MSB = uart_rx();
  uint8_t threshold_LSB = uart_rx();
  uint8_t threshold_decimal = uart_rx();
  uint8_t threshold_dec_MSB = uart_rx();
  uint8_t threshold_dec_LSB = uart_rx();
  uint8_t roof_behavior = uart_rx();
  
  // Construct the number
  uint16_t threshold_whole = threshold_MSB;
  threshold_whole = (threshold_whole << 8) | threshold_LSB;

  uint16_t threshold_dec_whole = threshold_dec_MSB;
  threshold_dec_whole = (threshold_dec_whole << 8) | threshold_dec_LSB;

  // Represented as float
  float threshold = threshold_whole + (threshold_dec_whole / 100);

  // Check to see which sensor it is for and save to the structures
  if (sensor == 2)
  {
    // Wind
    sensor_thresholds.wind_mph = threshold;
    roof_behaviors.wind_mph = roof_behavior;
  }
  else if (sensor == 3)
  {
    // UV
    sensor_thresholds.uv_index = threshold;
    roof_behaviors.uv_index = roof_behavior;
  }
  else if (sensor == 4)
  {
    // Temperature Min
    sensor_thresholds.min_temp = threshold;
    roof_behaviors.min_temp = roof_behavior;
  }
  else if (sensor == 5)
  {
    // Temperature Max 
    sensor_thresholds.max_temp = threshold;
    roof_behaviors.max_temp = roof_behavior;
  }
  
}

// Timer1 Interrupt to read all sensors
void poll_sensors()
{
   // Poll I2C Sensors
   // Can get the temperature measurement from this as well
   // If desired, uncomment line that says HERE in poll_hum() function
   // Then comment out the first ADC sensor poll with temp_ch
   current_measures.relative_humidity = poll_hum();
   current_measures.uv_index = poll_uv();
   
   // ADC Sensors
   current_measures.temperature = poll_temp();
   current_measures.rain = poll_rain( );
   
   // Wind is based on external interrupt so it takes care of itself
   
   // Calculate Relative Humidity with measured humidity and temperature
   current_measures.heat_index = calc_hi();

   // Check the values against thresholds
   check_thresholds();

   // Store averaged measurements
   average_measurements();
}

// DO NOT TOUCH HUMIDITY
void init_humidity()          
{
    // Humidity Sensor Address(s)
    // Updated 3/30 for SHT-85
    // Does a temperature reading and a humidity reading (IN THAT ORDER)
    // Command structured for medium repeatability 
    uint8_t addr_w = 0x88;
    uint8_t cmdMSB = 0x24;
    uint8_t cmdLSB = 0x0B;
    
    // START (S)
    twi_start();

    // Send Address check to make sure the ACK is received then send command
    twi_write(addr_w);

    if((TWSR & 0xF8) == 0x20)
    {
      uart_putstring("SLA+W NACK\n");
      // As long as not receiving an ACK resend
      //twi_write(addr_w);
    }

    // Send Write Command
    twi_write(cmdMSB);
    // Check Slave Response
    if((TWSR & 0xF8) != 0x28)
    {
      uart_putstring("CMD MSB NACK\n");
      return;
    }

    // Finish command
    twi_write(cmdLSB);
    if((TWSR & 0xF8) != 0x28)
    {
      uart_putstring("CMD LSB NACK\n");
      return;
    }
    // Stop the write/command section
    twi_stop();
}

// DO NOT TOUCH HUMIDITY
float poll_hum()       
{    
    // Send START again and read command 
    uint8_t addr_r = 0x89;
    twi_start();
    _delay_ms(1);
    twi_write(addr_r);

    if((TWSR & 0xF8) == 0x20)
    {
      uart_putstring("SLA+W NACK 2\n");
    }
    else if ((TWSR & 0xF8) == 0x28)
    {
      uart_putstring("SLA+W lost");
    }

    // First measurement is temperature two sperate 8-bits
    uint8_t upperT = twi_read_ack();
    _delay_ms(10);
    uint8_t lowerT = twi_read_ack();
    _delay_ms(10);
    uint8_t crcT = twi_read_ack();

    // Create the 16-bit RH number
    uint16_t sensorT = upperT;
    sensorT = sensorT << 8;
    sensorT &= 0xFF00;
    sensorT |= lowerT;

    // This gives you temperature
    uart_putstring("temp from H:\n");
    uart_write_uint16(floor(sensorT));  
    
    // Uncomment HERE for temperature not from ADC:
    // float temp = -49 * 315 (sensorT / 65535);
    // current_measures.temperature = temp;

    _delay_ms(1000);
    // Second measurement is humidity same format as temperature
    uint8_t upperH = twi_read_ack();
    _delay_ms(10);
    uint8_t lowerH = twi_read_ack();
    _delay_ms(10);
    uint8_t crcH = twi_read_nack();
    
    // Send STOP (P)
    twi_stop();

    // Create the 16-bit RH number
    uint16_t sensorRH = upperH;
    sensorRH = sensorRH << 8;
    sensorRH &= 0xFF00;
    sensorRH |= lowerH;

    // Equation for RH:
    // RH = 100 * sensorRH / 65535
    // Returns this in %
    float rh = 100 * (sensorRH / 65535);

    return rh;
}


void init_uv()
{
  
}

uint16_t poll_uv()
{
   uart_putstring("start L\n");
    // Send START again and read command 
    uint8_t addr_r = 0x89;
    twi_start();
    _delay_ms(1);
    twi_write(addr_r); 

    if((TWSR & 0xF8) == 0x20)
    {
      uart_putstring("SLA+W NACK 2\n");
    }
    else if ((TWSR & 0xF8) == 0x28)
    {
      uart_putstring("SLA+W lost");
    }

}

// ADC conversion and averaging for rain
int poll_rain()
{
  // Average ADC
  int i = 1;
  uint8_t rain, avg_rain = 0;
  
  while (i <= 5)
  { 
    int temp_rain = adc_get_conversion(RAIN_CH);
    if (temp_rain == 0)
    {
      // This doesnt make sense but sometimes the ADC needs to read the other to work 
      // so just call it
      float temp = poll_temp();
      continue;
    }
    i++;
    avg_rain += temp_rain;
  }
  rain = avg_rain / 5;

  // Vin = ADC * 3.3 / 256
  // = ADC * 3.3 / 256
  // = ADC * 0.01289
  float V = (float)rain * 0.01289;
  int rain_bool = 0;
  if (V < 2.0 )
  {
    rain_bool = 1;
  }

  return rain_bool;  
}

// ADC conversion and averaging for temp
float poll_temp()
{
  uint8_t temp = adc_get_conversion(TEMP_CH);

  // V = ADC * 3.3 / 256
  // = ADC * 3.3 / 256
  // = ADC * 0.0128906
  float V = (float)temp * 0.01289;
  
  // Vout = Vs * (Rbalance / Rtherm + R balance)
  // Rtherm = Rbalance * (Vs/Vout - 1)
  // Can do two different ways
  // With the voltages:
  // Rtherm = Rbalance * (3.3/V - 1)
  // Or with the ADC values        
  // Rtherm = Rbalance * (1023 / ADCmeasured - 1)
  
  float R_therm = R_BALANCE * (1023 / temp - 1);
  //uart_putstring("\nR_THERM\n");
  //uart_write_uint8(floor(R_therm));
  
  // Now calculate temperature
  // 1/T = 1/To + (1/B) * ln(R/Ro)
  // Ro is resistance at reference temp To  (Room temperature)
  // This is in degrees Celsius!!!!
  //float invert_temp = (1 / ROOM_TEMP) + (1 / TEMP_BETA) * log(R_therm / R_BALANCE);
  //float temp_final = 1 / invert_temp;
  float temp_kelvin = (TEMP_BETA * ROOM_TEMP) / (TEMP_BETA + 
                      (ROOM_TEMP * log(R_therm / R_BALANCE)));
  
  // Convert to C and F (whatever you want)
  float temp_final_c = temp_kelvin - 273.15;
  float temp_final_f = temp_final_c * 1.8 + 32;

  return temp_final_f;
}

// Wind functions were never tested
void init_wind()
{
  SampleRequired = false;
  Rotations = 0;
  // Set Wind Pin (A0 pin 78) to input and enable external interrupts for falling edge
  // Pin 6 Port/Pin PE4
  DDRE |= (0 << DDE4);
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

// Function to check the different thresholds compared to measurements
void check_thresholds()                 
{
  // 0 - Closed     1 - Open
  int roof_state = get_roof_state();

  if (current_measures.temperature > sensor_thresholds.max_temp)
  {
    uart_putstring("Temperature max triggered");
    if (roof_state != roof_behaviors.max_temp)
      change_roof_state(roof_behaviors.max_temp);
  }
  else if (current_measures.temperature < sensor_thresholds.min_temp)
  {
    uart_putstring("Temperature min triggered");
    if (roof_state != roof_behaviors.min_temp)
      change_roof_state(roof_behaviors.min_temp);
  }
  else if (current_measures.wind_mph > sensor_thresholds.wind_mph)
  {
    uart_putstring("Wind triggered");
    if (roof_state != roof_behaviors.wind_mph)
      change_roof_state(roof_behaviors.wind_mph);
  }
  else if (current_measures.relative_humidity > sensor_thresholds.relative_humidity)
  {
    uart_putstring("Humidity triggered");
    if (roof_state != roof_behaviors.relative_humidity)
      change_roof_state(roof_behaviors.relative_humidity);
  }
  else if (current_measures.heat_index > sensor_thresholds.heat_index)
  {
     uart_putstring("Heat index triggered");
     if (roof_state != roof_behaviors.heat_index)
      change_roof_state(roof_behaviors.heat_index);
  }
  else if (current_measures.uv_index > sensor_thresholds.uv_index)
  {
    uart_putstring("UV Index triggered");
    if (roof_state != roof_behaviors.uv_index)
      change_roof_state(roof_behaviors.uv_index);
  }
  else if (current_measures.rain)
  {
    uart_putstring("Rain triggered");    
    if (roof_state != roof_behaviors.rain)
      change_roof_state(roof_behaviors.rain);
  }
}

int get_roof_state()
{
  return ROOF_STATUS;
}

void change_roof_state(int new_setting)
{
  // New setting will be 0/1 and can use that to be able to tell direction of the actuators
  ROOF_STATUS = new_setting
  
  // PWM enable outputs 5V on the pin
  PWM0_enable();

  // Set delay here or put in proximity 
  _delay_ms(3000);

  // PWM disbale shuts the 5V off
  PWM0_disable();
  
}

void average_measurements()
{
  // Add the new measurements to create average values that will be sent 
  // to the phone when next connected

  avg_measures.temperature = (current_measures.temperature + avg_measures.temperature) / 2;
  avg_measures.wind_mph = (current_measures.wind_mph + avg_measures.wind_mph) / 2;
  avg_measures.relative_humidity = (current_measures.relative_humidity + avg_measures.relative_humidity) / 2;
  avg_measures.heat_index = (current_measures.heat_index + avg_measures.heat_index) / 2;
  avg_measures.uv_index = (current_measures.uv_index + avg_measures.uv_index) / 2;
  avg_measures.rain = (current_measures.rain + avg_measures.rain) / 2;

  return; 
}
/********************************
 *        PWM Functions
 ********************************/
void PWM0_enable()
{
  // Write the Actuator pin to high
  analogWrite(ACTUATOR_PIN,255);
}

void PWM0_disable()
{
  // Write the Actuator pin to low
  analogWrite(ACTUATOR_PIN,0);
}

/********************************
 *        ADC Functions
 ********************************/

void init_adc()
{
  ADMUX = (1 << REFS0)                        /* AVCC 3.3V Ref*/
          | (1 << ADLAR);                     /* Right Adjusted Result */

  ADCSRA = (1 << ADEN)                                      /* ADC: enabled */
           | (0 << ADATE)                                   /* Auto Trigger: disabled */
           | (0 << ADIE)                                    /* ADC Interrupt: disabled */
           | (1<< ADPS0) | (1 << ADPS1) | (1 << ADPS2);     /* /128 for 128kHz*/
  ADCSRB = (0x00 << ADTS0)                                  /* Free Running mode */
           | (0 << ACME);                                   /* Analog Comparator Multiplexer: disabled */

}

uint8_t adc_get_conversion(uint8_t channel)
{
  // double checked that channel comes in properly = passed
  adc_start_conversion(channel);
  _delay_ms(1);
  
  while(!adc_done());
  uart_putstring("result:\n");
  
  uint8_t result = ADC;
  uart_write_uint8(result);
  
  // Clear the flag
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
  //return (ADCSRA & (1 << ADSC));
  return ((ADCSRA & (1 << ADIF)));  // ADIF becomes 1 when done so F when not done 
}

uint16_t adc_result()
{
  // Not applicable in 4/10 implementation
  return ADC;
}

/********************************
 *        I2C Functions
 ********************************/
void init_twi()
{
    // Set SCL to 100kHz
    TWSR = 0x00;
    TWBR = 0x20;
    
    // Enable TWI
    TWCR = (1<<TWEN);
    
    return;
}

void twi_start()
{
    // Send start signal
    TWCR = (1 << TWINT) | (1<<TWSTA) | (0 << TWSTO) | (1<<TWEN);
    while (!(TWCR & (1 <<TWINT)));
    return;
}

void twi_stop()
{
    // Send stop signal
    TWCR = (1 << TWINT) | (0 << TWSTA) | (1 << TWSTO) | (1 <<TWEN);
    return;
}

void twi_write(uint8_t data)
{    
    // Set the data to be transmitted and wait for completion
    TWDR = data;
    
    // Clear START bit
    TWCR &= ~(1 << TWSTA);
    // Clearing INT flag starts operation
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 <<TWINT)));
    return;
}

uint8_t twi_read_ack()
{
    // Read byte with ACK
    TWCR |= (1 << TWEA);                  // Set up ACK
    TWCR |= (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 <<TWINT)));
    return TWDR;
}

uint8_t twi_read_nack()
{
    // Read byte with NACK
    TWCR &= ~(1 << TWEA);                   // Set up NACK
    TWCR |= (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 <<TWINT)));
    return TWDR;
}

uint8_t twi_status_NA()
{
    // No longer used but here are the different possible statuses in TWSR
    // Get the status of TWI
    // In Master Mode:
    // 0x18: SLA+W followed by ACK
    // 0x20: SLA+W followed by NACK
    // 0x38: Lost SLA+W 
    // 0x28: Data transmitted followed by ACK
    // 0x30: Data transmitted followed by NACK

}

/******************************************************************************************/
/******************************************************************************************/
void init_values()
{
  // Initialize the thresholds for default
  sensor_thresholds.max_temp = 100;
  sensor_thresholds.min_temp = 32;
  sensor_thresholds.wind_mph = 28;
  sensor_thresholds.relative_humidity = 82; 
  sensor_thresholds.heat_index = 110;
  sensor_thresholds.uv_index = 11;

  // Initialize the behaviors for default
  roof_behaviors.max_temp = 1;
  roof_behaviors.min_temp = 0;
  roof_behaviors.wind_mph = 0;
  roof_behaviors.relative_humidity = 0;
  roof_behaviors.heat_index = 0;
  roof_behaviors.uv_index = 0;
  roof_behaviors.rain = 0; 
}

/******************************************************************************************/
// Main functions, setup runs once at the start, loop runs continously. Loop should be bare
void setup()
{
  // For LED on board, and for testing
  DDRB |= (1 <<DDB7); 

  // Set pins for PWM
  DDRG |= (1 << DDG5);

  // Set pins for ADC
  DDRF = 0x00;
  
  // Disable Interrupts
  cli();

  // Initialize all the values in structures and the roof 
  ROOF_STATUS = 1;   // Starts open
  init_values();

  // Initialize the Timers
  init_t0();
  init_t1();

  // Initialize the I2C
  init_twi();
  
  // Initialize the USART 
  // Baud is 9600 in serial monitor 
  uint8_t ubrr = MYUBRR;        // Check macros for definition
  init_uart0(ubrr);             // Bluetooth

  // Initialize the ADC
  // Starts first conversion which will take 25 clock cycles at 128kHz
  init_adc(); 
  
  // Initilaize Sensors
  init_uv();
  init_wind();
  
  // Set Global Inerrupt Enable
  sei();

}

// Continuously runs on the board, minimal code as possible
void loop()
{


}
// Scroll down for UART functions







/*******************************
 *    Uart Helper Functions
 *******************************/
void uart_putstring(char * strptr)
{
  while(*strptr != 0x00)
  {
    uart_tx(*strptr);
    strptr++;
  }
}

void uart_write_uint32(uint32_t n)
{
   int digit;
   if(n >= 100000000)
   {
     digit = (n/100000000) % 10;
     uart_tx(digit + '0');
   }
   if(n >= 10000000)
   {
     digit = (n/10000000) % 10;
     uart_tx(digit + '0');
   }
   if(n >= 1000000)
   {
     digit = (n/1000000) % 10;
     uart_tx(digit + '0');
   }
   if(n >= 100000)
   {
     digit = (n/100000) % 10;
     uart_tx(digit + '0');
   }
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

void uart_write_uint8(uint8_t n)
{
   int digit;
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
