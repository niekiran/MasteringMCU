/*
  Serial Event example

  When new serial data arrives, this sketch adds it to a String.
  When a newline is received, the loop prints the string and clears it.

  A good test for this is to try it with a GPS receiver that sends out
  NMEA 0183 sentences.

  NOTE: The serialEvent() feature is not available on the Leonardo, Micro, or
  other ATmega32U4 based boards.

  created 9 May 2011
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/SerialEvent
*/
#include <SPI.h>
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
uint8_t t_buffer[200];
uint32_t cnt = 0;
//Initialize SPI slave.
void SPI_SlaveInit(void) 
{ 
  // Initialize SPI pins.
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);
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
}
  // The setup() function runs after reset.
void setup() 
{
  // Initialize serial for troubleshooting.
  Serial.begin(9600);
  

  
  // Initialize SPI Slave.
  SPI_SlaveInit();
  
  Serial.println("Slave Initialized");
}

void loop() {
  //1. fist make sure that ss is low . so lets wait until ss is low 
  //Serial.println("Slave waiting for ss to go low");
  while(digitalRead(SS) );

 while(stringComplete){
 //Serial.println(inputString);
 Serial.println((char*)t_buffer);
//SPI_SlaveReceive();
SPI_SlaveTransmit(0xf1);
for(uint32_t i = 0 ; i < cnt ; i++)
{
 // SPI_SlaveReceive();
  SPI_SlaveTransmit(t_buffer[i]);
}
cnt = 0;
 stringComplete = false;
 Serial.println("end");
 }

 
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
   // inputString += inChar;
    t_buffer[cnt++] = inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\r') {
      t_buffer[cnt++] = '\0';
      stringComplete = true;
    }
  }
}
