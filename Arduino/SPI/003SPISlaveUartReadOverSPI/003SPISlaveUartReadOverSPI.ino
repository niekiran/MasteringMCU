#include <SPI.h>

#define MAX_LEN 500

bool msgComplete = false;  // whether the string is complete
uint8_t userBuffer[MAX_LEN];
uint32_t cnt = 0;

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
 
void setup() 
{
  // Initialize serial for troubleshooting.
  Serial.begin(9600);
  
  // Initialize SPI Slave.
  SPI_SlaveInit();

  pinMode(8, INPUT_PULLUP);
  //digitalWrite(8,LOW);

  Serial.println("Slave Initialized");
}

void notify_controller(void)
{
  pinMode(8,OUTPUT);
  digitalWrite(8,HIGH);
  delayMicroseconds(50);
  digitalWrite(8,LOW);
}


void loop() {
  
  Serial.println("Type anything and send...");

  while(!msgComplete){
    if (Serial.available()) {
      //Read a byte of incoming serial data.
      char readByte = (char)Serial.read();
      //Accumalate in to the buffer
      userBuffer[cnt++] = readByte;
      if(readByte == '\r' || ( cnt == MAX_LEN)){
        msgComplete = true;
        userBuffer[cnt -1 ] = '\0'; //replace '\r' by '\0'
      }
    }
  }
  
  Serial.println("Your message...");
  Serial.println((char*)userBuffer);

  
   notify_controller();

  /*Transmit the user buffer over SPI */
  for(uint32_t i = 0 ; i < cnt ; i++)
  {
    SPI_SlaveTransmit(userBuffer[i]);
  }
  cnt = 0;
  msgComplete = false;
  Serial.println("Message sent...");

  while(!digitalRead(SS));
  Serial.println("Master ends communication");
 
}
