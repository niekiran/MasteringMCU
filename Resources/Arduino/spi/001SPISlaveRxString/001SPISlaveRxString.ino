/* SPI Slave Demo

 *
 * SPI pin numbers:
 * SCK   13  // Serial Clock.
 * MISO  12  // Master In Slave Out.
 * MOSI  11  // Master Out Slave In.
 * SS    10  // Slave Select . Arduino SPI pins respond only if SS pulled low by the master
 *
 
 */
#include <SPI.h>
#include<stdint.h>
#define SPI_SCK 13
#define SPI_MISO 12
#define SPI_MOSI 11
#define SPI_SS 10

#define MAX_I2C_PAYLOAD_LENGTH  500

char dataBuff[MAX_I2C_PAYLOAD_LENGTH];
//Initialize SPI slave.
void SPI_SlaveInit(void) 
{ 
  // Initialize SPI pins.
  pinMode(SCK, INPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, OUTPUT);
  pinMode(SS, INPUT);

  // Enable SPI as slave.
  SPCR = (1 << SPE);
}

//This function returns SPDR Contents 
uint8_t SPI_SlaveReceive(void)
{
  /* Wait for reception complete */
  //Serial.println("in SPI_SlaveReceive - waiting for reception complete");
  while(!(SPSR & (1<<SPIF)));
  //Serial.println("reception complete !!!");
  /* Return Data Register */
  return SPDR;
}

//send one byte of data 
void SPI_SlaveTransmit(char data)
{
  /* Start transmission */
  SPDR = data;
  /* Wait for transmission complete */
  while(!(SPSR & (1<<SPIF)));
}


// The setup() function runs right after reset.
void setup() 
{
  // Initialize serial communication 
  Serial.begin(9600);

  // Initialize SPI Slave.
  SPI_SlaveInit();
  
  Serial.println("Slave Initialized");
}


// The loop function runs continuously after setup().
void loop() 
{
  uint32_t i;
  uint8_t dataLen8;
  //uint16_t dataLen16; // To support more than 255 bytes payload transfer in a single chunk. currently not in use.
  Serial.println("\nSlave waiting for SS to go low");

  while(digitalRead(SS));
  // Important !!! Avoid prints here! Enabling the print in the next line delays the slave Arduino and this cause it to miss data bytes sent from the master unless the master will add some delay between sending the length and the payload.
  //Serial.println("SS is now Low"); 

  //1. read the length  
  //dataLen16 = (uint16_t)( SPI_SlaveReceive() | (SPI_SlaveReceive() << 8) );
  //Serial.println(String(dataLen,HEX));
  dataLen8 = SPI_SlaveReceive();
  //Serial.print("Received Data Len: ");
  //Serial.println(String(dataLen8, DEC));
  //Serial.println("Following will be the Received Data: ");

  for(i = 0 ; i < dataLen8 ; i++ ) {
    dataBuff[i] = SPI_SlaveReceive();
  }
  dataBuff[i] = '\0';

  Serial.print("Rcvd: '");
  Serial.print(dataBuff);
  Serial.print("'. Length: ");
  Serial.println(dataLen8);

//  Serial.println("Slave waiting for ss to go high");
//  while(!digitalRead(SS));
//  Serial.println("SS HIGH\n\n");
}
