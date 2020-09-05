/*

 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select
 *

 */

#include <SPI.h>

const byte led = 9;           // Slave LED digital I/O pin.

boolean ledState = HIGH;      // LED state flag.

uint8_t dataBuff[255];

uint8_t board_id[10] = "ARDUINOUNO";

#define NACK  0xA5
#define ACK   0xF5


//command codes
#define COMMAND_LED_CTRL          0x50
#define COMMAND_SENSOR_READ       0x51
#define COMMAND_LED_READ          0x52
#define COMMAND_PRINT             0x53
#define COMMAND_ID_READ           0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0   0
#define ANALOG_PIN1   1
#define ANALOG_PIN2   2
#define ANALOG_PIN3   3
#define ANALOG_PIN4   4

//Initialize SPI slave.
void SPI_SlaveInit(void) 
{ 
  // Initialize SPI pins.
  pinMode(SCK,  INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS,   INPUT);
  //make SPI as slave
  
  // Enable SPI as slave.
  SPCR = (1 << SPE);
}

//This function returns SPDR Contents 
uint8_t SPI_SlaveReceive(void)
{
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)));
  /* Return Data Register */
  return SPDR;
}


//sends one byte of data 
void SPI_SlaveTransmit(uint8_t data)
{
  /* Start transmission */
  SPDR = data;
  
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
  Serial.print("SPISlave sent byte: 0x");
  Serial.println(data, HEX);
}
  

// The setup() function runs after reset.
void setup() 
{
  // Initialize serial for troubleshooting.
  Serial.begin(9600);
  
  // Initialize slave LED pin.
  pinMode(led, OUTPUT);
  
  digitalWrite(led,LOW);
  
  // Initialize SPI Slave.
  SPI_SlaveInit();
  
  Serial.println("Slave Initialized");
}


byte checkCmdOpcode(byte cmdOpcode)
{
  Serial.print("command received: 0x");
  Serial.println(cmdOpcode, HEX);
  if ((cmdOpcode == COMMAND_LED_CTRL) ||
      (cmdOpcode == COMMAND_SENSOR_READ) ||
      (cmdOpcode == COMMAND_LED_READ) ||
      (cmdOpcode == COMMAND_PRINT) ||
      (cmdOpcode == COMMAND_ID_READ)) {
    return ACK;
  }
  // Illegal opcode received.
  return NACK;
}

// The loop function runs continuously after setup().
void loop() 
{
  byte data, cmdOpcode, len, ackornack = NACK;
  
  //1. fist make sure that ss is low . so lets wait until ss is low 
  Serial.println("Slave waiting for ss to go low");
  while(digitalRead(SS) );
  Serial.println("Slave ss is now low");
  
  //2. now lets wait until rx buffer has a byte
  cmdOpcode = SPI_SlaveReceive();
  ackornack = checkCmdOpcode(cmdOpcode);
  
  SPI_SlaveTransmit(ackornack);
  
  len = SPI_SlaveReceive(); //dummy byte
  
  if(cmdOpcode == COMMAND_LED_CTRL)
  {
    //read 2 more bytes pin number and value 
    uint8_t pin = SPI_SlaveReceive(); 
    uint8_t value = SPI_SlaveReceive(); 
    Serial.print("RCVD:COMMAND_LED_CTRL on pin: ");
    Serial.print(pin);
    if (value == 1) {
      Serial.println(" ON (1)");
    } else if (value == 0) {
      Serial.println(" OFF (0)");
    } else {
      Serial.print(" -- ERROR: value unexpected: 0x");
      Serial.println(value, HEX);
    }
    if(value == (uint8_t)LED_ON)
    {
      digitalWrite(pin,HIGH);
    }else if (value == (uint8_t) LED_OFF)
    {
      digitalWrite(pin,LOW);
    }
  }else if ( cmdOpcode == COMMAND_SENSOR_READ)
  {
    //read analog pin number 
    uint16_t aread;
    uint8_t pin = SPI_SlaveReceive(); 
    //pinMode(pin+14, INPUT_PULLUP);
    uint8_t val;
    aread = analogRead(pin+14);
    val = map(aread, 0, 1023, 0, 255);
    
    SPI_SlaveTransmit(val);
  
    val = SPI_SlaveReceive(); //dummy read
    
    Serial.print("RCVD:COMMAND_SENSOR_READ from pin");
    Serial.print(pin);
    Serial.print(" Analog port value: ");
    Serial.print(aread);
    Serial.print(", ");
    Serial.println(val);
  
    
  
  }else if ( cmdOpcode == COMMAND_LED_READ)
  {
    uint8_t pin = SPI_SlaveReceive(); 
    uint8_t val = digitalRead(pin);
    SPI_SlaveTransmit(val);
    val = SPI_SlaveReceive(); //dummy read
    Serial.println("RCVD:COMMAND_LED_READ");
  
  }else if ( cmdOpcode == COMMAND_PRINT)
  {
    uint8_t len = SPI_SlaveReceive(); 
    for(int i=0 ; i < len ; i++)
    {
      dataBuff[i] = SPI_SlaveReceive();
    }
    Serial.println((char*)dataBuff);
    
    Serial.println("RCVD:COMMAND_PRINT");
  
  }else if ( cmdOpcode == COMMAND_ID_READ)
  {
    for(int i=0 ; i < strlen(board_id) ; i++)
    {
      SPI_SlaveTransmit(board_id[i]);
    }
      SPI_SlaveReceive();
    Serial.println("RCVD:COMMAND_ID_READ");
  }
 

}
