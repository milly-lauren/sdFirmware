/*
 * Final version of SD Firmware Software
 * To demonstrate sensor readings ONLY 
 * Author: Lauren Miller
 * Created:4/11/2020 
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
 *         UV Functions
 ********************************/
typedef uint8_t BYTE;
typedef uint16_t WORD;
#define VEML6070_ADDR_ARA (0x18 >> 1)
#define VEML6070_ADDR_CMD (0x70 >> 1)
#define VEML6070_ADDR_DATA_LSB (0x71 >> 1)
#define VEML6070_ADDR_DATA_MSB (0x73 >> 1)

// VEML6070 command register bits
#define VEML6070_CMD_SD       0x01
#define VEML6070_CMD_IT_0_5T  0x00
#define VEML6070_CMD_IT_1T    0x04
#define VEML6070_CMD_IT_2T    0x08
#define VEML6070_CMD_IT_4T    0x0C
#define VEML6070_CMD_WDM      0x02
#define VEML6070_CMD_DEFAULT (VEML6070_CMD_WDM | VEML6070_CMD_IT_1T)

enum RISK_LEVEL{low, moderate, high, very_high, extreme};

int cmd = VEML6070_CMD_DEFAULT;
// word is 2 bytes
WORD uvs_step;
RISK_LEVEL risk_level;


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


ISR(USART0_RX_vect)
{
    uint8_t cmd = UDR0;
    if (cmd == '1')
    {
      // Temperature
      float temp = poll_temp();
      uart_putstring("Temperature:");
      uart_write_uint16(floor(temp));
      uart_putstring("\n");
    }
    else if (cmd == '2')
    {
      // Humidity (Also has temperature if needed)
      uint16_t hum = poll_hum();
      uart_putstring("Humidity:");
      uart_write_uint16(hum);
      uart_putstring("\n");
    }
    else if (cmd == '3')
    {
      // Rain
      int rain = poll_rain(); 
      // Closer to 1V means more wet (add this as comment to video bc cant send float)
      // Use rain value to calculate the Vin 
      // Vin = ADC * 3.3 / 256
      // = ADC * 3.3 / 256
      // = ADC * 0.01289
      uart_putstring("Rain V:");
      uart_write_uint8(rain);
      uart_putstring("\n");      
    }
    else if (cmd == '4')
    {
      // Wind will print the MPH in the poll_wind() function
      poll_wind();
    }
    else if (cmd == '5')
    {
      // UV Index
      uint16_t uv = poll_uv();
      uart_putstring("UV Index:");
      uart_write_uint16(floor(uv));
      uart_putstring("\n");
    }
    else
    {
      uart_putstring("CMD ERROR\n");
    }
}

/********************************
 *        Sensor Functions
 ********************************/
// DO NOT TOUCH HUMIDITY
void init_humidity()          //  ****************************** Humidity Functions ********************
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
uint16_t poll_hum()       
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

    uart_putstring("Temperature\n");
    uart_write_uint16(floor(sensorT));
    uart_putstring("\n");    

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

    return sensorRH;
}

void init_uv()
{
   initialize_VEML6070();
}

uint16_t poll_uv()
{
  uvs_step = read_uvs_step();
  risk_level = convert_to_risk_level(uvs_step);
  return risk_level;
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
    uart_putstring("Rain sensed\n");
    rain_bool = 1;
  }

  return rain;  
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
    float mph = Rotations * 0.75;

    // Write the floor of MPH
    uart_putstring("Wind MPH:");
    uart_write_uint16(floor(mph));
    uart_putstring("\n");
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
  
  //uint16_t result = adc_result();

  // FOR TESTING ONLY
  //uint16_t result = adc_result_test();


  
  uint8_t result = ADC;
  uart_write_uint8(result);
  
  
  // Clear the flag
  ADCSRA |= (1 << ADIF);
  
  return result;
}

void adc_start_conversion(uint8_t channel)
{
  // mux and channel checked here = passed
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
  // These writes always returned 0 (4/4/20)
  //uart_write_uint8(ADCL);
  //uart_write_uint8(ADCH);
  //return ((ADCL | ADCH) << 8);
  return ADCW;
}

uint16_t adc_result_test()
{
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
    // Get the status of TWI
    // In Master Mode:
    // 0x18: SLA+W followed by ACK
    // 0x20: SLA+W followed by NACK
    // 0x38: Lost SLA+W 
    // 0x28: Data transmitted followed by ACK
    // 0x30: Data transmitted followed by NACK

}

/**********************************************************************************************************************************/
/**********************************************************************************************************************************/
/**********************************************************************************************************************************/

void setup() 
{
  // Set pins for PWM
  DDRG |= (1 << DDG5);

  // Set pins for ADC
  DDRF = 0x00;
  
  // Disable Interrupts
  cli();

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
  init_wind();
  //init_uv();

  // Set Global Inerrupt Enable
  sei();

}

void loop() 
{
  // Put your main code here, to run repeatedly:
  // Scroll down for UART and UV functions
}



/*****************
 *      UV
 *****************/
void initialize_VEML6070(void)
{
  // Read ARA to clear interrupt
  BYTE address;
  VEML6070_read_byte(VEML6070_ADDR_ARA, &address);
  // Initialize command register
  VEML6070_write_byte(VEML6070_ADDR_CMD, cmd);
  delay(200);
}
void enable_sensor(void)
{
  cmd &= ~VEML6070_CMD_SD;
  VEML6070_write_byte(VEML6070_ADDR_CMD, cmd);
}
void disable_sensor(void)
{
  cmd |= VEML6070_CMD_SD;
  VEML6070_write_byte(VEML6070_ADDR_CMD, cmd);
}
WORD read_uvs_step(void)
{
  BYTE lsb, msb;
  WORD data;
  VEML6070_read_byte(VEML6070_ADDR_DATA_MSB, &msb);
  VEML6070_read_byte(VEML6070_ADDR_DATA_LSB, &lsb);
  data = ((WORD)msb << 8) | (WORD)lsb;
  return data;
}

RISK_LEVEL convert_to_risk_level(WORD uvs_step)
{
  WORD risk_level_mapping_table[4] = {2241, 4482, 5976, 8217};
  WORD i;
  for (i = 0; i < 4; i++)
  {
    if (uvs_step <= risk_level_mapping_table[i])
    {
      break;
    }
  }
  return (RISK_LEVEL)i;
}

void VEML6070_read_byte(WORD addr, BYTE *data)
{
  twi_start();
  twi_write(addr);

  // Send start again and read
  twi_start();
  *data = twi_read_nack();
}

void VEML6070_write_byte(WORD addr, BYTE data)
{
  twi_start();
  twi_write(addr);
  
  if((TWSR & 0xF8) == 0x20)
  {
    uart_putstring("SLA+W NACK 2\n");
  }
  else if ((TWSR & 0xF8) == 0x28)
  {
    uart_putstring("SLA+W lost");
  }
    
  twi_write(data);
  twi_stop(); 
}


/*
 * Uart Helper Functions
 */
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
