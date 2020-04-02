// To test bluetooth
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

/********************************
 *      Clock Constants
 *******************************/
#define FOSC        8388608
#define BAUD        4800
#define MYUBRR      FOSC/16/BAUD - 1

/********************************
 *       Hardware Constants
 ********************************/
#define TEMP_CH               0x00
#define RAIN_CH               0x01

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

float poll_humidity()       //  ****************************** Humidity Functions ********************
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
    for (int i = 0; i < 65535; i++){}

    while (twi_status() != 0x18)
    {
      // As long as not receiving an ACK resend
      twi_write(addr_w);
      for (int i = 0; i < 65535; i++){}
    }

    // Send Write Command
    twi_write(cmdMSB);
    twi_write(cmdLSB);
    
    
    // Send START again and read command 
    uint8_t addr_r = 0x89;
    twi_start();
    twi_write(addr_r);

    // First measurement is temperature two sperate 8-bits
    uint8_t upperT = twi_read_ack();
    uint8_t lowerT = twi_read_ack();
    uint8_t crcT = twi_read_ack();

    // Second measurement is humidity same format as temperature
    uint8_t upperH = twi_read_ack();
    uint8_t lowerH = twi_read_ack();
    uint8_t crcH = twi_read_nack();

    // Send STOP (P)
    twi_stop();

    // Create the 16-bit RH number
    uint16_t sensorRH = upperH;
    sensorRH = sensorRH << 8;
    sensorRH &= 0xFF00;
    sensorRH |= lowerH;
        
    // Calculate RH (Result is %)
    float humidity = 100 * sensorRH / (65535);
    return humidity;
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
    TWCR = (1 << TWINT) | (1<<TWSTA) | (0 << TWSTO) | (1<<TWEN);
    while((TWCR & (1<<TWINT)) == 0);
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
    // In Master Mode:
    // 0x18: SLA+W followed by ACK
    // 0x20: SLA+W followed by NACK
    // 0x38: Lost SLA+W 
    
    uint8_t status;
    status = TWSR & 0xF8;
    return status;
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
  
  // Disable Interrupts
  cli();

  // Initialize the I2C
  init_twi();
  
  // Initialize the USART 
  // Baud is 9600 in serial monitor 
  uint8_t ubrr = MYUBRR;        // Check macros for definition
  init_uart0(ubrr);             // Bluetooth
  //init_aurt1(ubrr);           // Computer Uart 

  // Initialize the ADC
  // Starts first conversion which will take 25 clock cycles at 128kHz
  init_adc(); 
  
   // Initilaize Barometrice Pressure
  init_bp();

  // Set Global Inerrupt Enable
  sei();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (UCSR0A & (1 << RXC0))
    {
      uint8_t cmd = uart_rx();
      if (strcmp(cmd,"h") == 0)
      { 
        // Poll humidity (checking i2c)
        float hum = poll_humidity();
        uart_tx(hum);
      }
      else if (strcmp(cmd,"b") == 0)
      {
        // Poll barometric pressure (checking i2c)
        float bp = poll_bp();
      }
      else if (strcmp(cmd,"t") == 0)
      {
        // Temperature Measurement (checking ADC)
        uint8_t temp = adc_get_conversion(TEMP_CH);
      }
      else if (strcmp(cmd,"r") == 0)
      {
        uint8_t rain = adc_get_conversion(RAIN_CH);
      }
      else
      {
        uart_tx(cmd);
      }
    }
}
