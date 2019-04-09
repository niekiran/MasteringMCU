// Wire Master Transmitter and Receiver 
//Uno, Ethernet A4 (SDA), A5 (SCL)
#include <Wire.h>

// Include the required Wire library for I2C<br>#include <Wire.h>
int LED = 13;

uint8_t rcv_buf[32];

int data_len=0;
#define SLAVE_ADDR 0x69

void setup() {
    Serial.begin(9600);
    
  // Define the LED pin as Output
  pinMode (LED, OUTPUT);
  
  // join i2c bus (address optional for master)
  Wire.begin(); 

   
  
}


void loop() {

  Serial.println("Arduino Master");
  Serial.println("Send character \"s\" to begin");
  Serial.println("-----------------------------");

   while(!Serial.available());
  char in_read=Serial.read();

  while(in_read != 's');

  Serial.println("Starting..");

  Wire.beginTransmission(SLAVE_ADDR);
  
  Wire.write(0X51); //Send this command to read the length
  Wire.endTransmission();


  Wire.requestFrom(SLAVE_ADDR,1); // Request the transmitted two bytes from the two registers

  if(Wire.available()) {  // 
    data_len = Wire.read(); // Reads the data 
  }
  Serial.print("Data Length:");
  Serial.println(String(data_len,DEC));

  Wire.beginTransmission(SLAVE_ADDR);
  
  Wire.write(0X52); //Send this command to ask data
  Wire.endTransmission();

  Wire.requestFrom(SLAVE_ADDR,data_len);

  uint32_t i=0;
  for( i =0; i <= data_len ; i++)
  {
    if(Wire.available()) {  // 
      rcv_buf[i] = Wire.read(); // Reads the data 
    }
  }
  rcv_buf[i] = '\0';

  Serial.print("Data:");
  Serial.println((char*)rcv_buf);
  Serial.println("*********************END*********************");
}
