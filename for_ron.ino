// To test bluetooth
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

ISR(USART0_RX_vect)
{
  // UART RX Complete Vector
  uint8_t cmd = UDR0;
      if (cmd == '1')
      {        
        // Poll humidity (checking i2c)
        uint16_t hum = poll_humidity();
        uart_putstring("RH no in %: \n");
        uart_write_uint16(hum);
        /* TO DO:
         *  we cant use the % value bc of the data type mismatch so we have 
         *  to convert the RH% given by the user into the sensor value 
         *  Srh = RH * 65535 / 100 
         *  THIS most likely will need to be done on the phone and use floor() for non float
         */
      }
      else if (cmd =='2')
      {
        uart_putstring("pressure\n");
        // Poll barometric pressure (checking i2c)
        float bp = 0 ;//poll_bp();
        uart_tx(bp);
      }
      else if (cmd =='3')
      {


        
        uart_putstring("\n\ntemperature\n");
        // Temperature Measurement (checking ADC)
        uint8_t temp = adc_get_conversion(TEMP_CH);
        //temp = temp >> 2;
        
        uart_putstring("\nreturn val\n");
        uart_write_uint8(temp);


        
      }
      else if (cmd == '4')
      {
        uart_putstring("rain\n");
        uint8_t rain = adc_get_conversion(RAIN_CH);
        uart_tx(rain);
      }
      else
      {
        uart_tx(cmd);
      }
}


/********************************
 *        Sensor Functions
 ********************************/
void init_bp()            //  ****************************** BP Functions ********************
{
    // Function used to initialize the pressure sensor
    // Set mode to forced, every time set to forced = measurement
    // THIS CODE IS ASSUMING SDO IS CONNECTED TO V (NOT GROUND)
    
    // Slave address
    uint8_t addr = 0xEE;     
            
    twi_start();
    twi_write(addr);

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
    
    // Send START (S)
    twi_start();

    // Send address
    twi_write(addr);
    twi_write(p_LSB);
    
    // Send second START command
    // NOW address is in write mode
    addr = 0xEF;
    twi_start();
    twi_write(addr);
    
    // Read the data for pressure and stop 
    uint8_t upperP = twi_read_ack();
    uint8_t lowerP = twi_read_nack();
    
    // Send STOP (P)
    twi_stop();

    
    // Equations
    // Create the 16-bit RH number
    uint16_t sensorP = upperP;
    sensorP = sensorP << 8;
    sensorP &= 0xFF00;
    sensorP|= lowerP;

    // To do: BP equations

    return sensorP;
}

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

uint16_t poll_humidity()       
{    
    uart_putstring("starting hum poll..\n");
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

    uart_putstring("reading H\n");
    _delay_ms(1000);
    // Second measurement is humidity same format as temperature
    uint8_t upperH = twi_read_ack();
    _delay_ms(10);
    uint8_t lowerH = twi_read_ack();
    _delay_ms(10);
    uint8_t crcH = twi_read_nack();
    
    // Send STOP (P)
    twi_stop();
    uart_putstring("stop\n");
    // Create the 16-bit RH number
    uint16_t sensorRH = upperH;
    sensorRH = sensorRH << 8;
    sensorRH &= 0xFF00;
    sensorRH |= lowerH;

    return sensorRH;
}

/********************************
 *        ADC Functions
 ********************************/

void init_adc()
{
  ADMUX = (1 << REFS0)                        /* AVCC 3.3V Ref*/
          | (1 << ADLAR);                     /* Right Adjusted Result */

  ADCSRA = (1 << ADEN)                        /* ADC: enabled */
           | (0 << ADATE)                     /* Auto Trigger: disabled */
           | (0 << ADIE)                      /* ADC Interrupt: disabled */
           | (1 << ADPS1) | (1 << ADPS2);     /* /64 for 128kHz*/
  ADCSRB = (0x00 << ADTS0)                    /* Free Running mode */
           | (0 << ACME);                     /* Analog Comparator Multiplexer: disabled */

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
  uint8_t result = ADCH;
  uart_write_uint8(result);
  
  // Vin = ADC * Vref / 1024
  // = ADC * 1.1 / 1024
  // = ADC * 0.00107
  float Vin = (float)result * 0.00107;
  
  if (Vin > 0)
    uart_putstring(" T\n");
  else 
    uart_putstring(" F\n");
  
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

uint8_t adc_result_test()
{
  return ADCH;
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
void setup() 
{
  // RX is input
  DDRE |= (0 << DDE0);
  // TX is output
  DDRE |= (1 << DDE1); 

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
  //init_uart1(ubrr);           // Computer Uart 

  // Initialize the ADC
  // Starts first conversion which will take 25 clock cycles at 128kHz
  init_adc(); 
  //uint16_t first_conv = adc_get_conversion(TEMP_CH);
  
  // Initilaize Sensors
  //init_bp();
  _delay_ms(35);
  //init_humidity();

  // Set Global Inerrupt Enable
  sei();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
