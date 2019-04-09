// Wire Slave Transmitter and receiver
//Uno, Ethernet A4 (SDA), A5 (SCL)
#include <Wire.h>

// Include the required Wire library for I2C<br>#include <Wire.h>
int LED = 13;
uint8_t active_command = 0xff,led_status=0;
char name_msg[32] = "Welcome to FastBit EBA\n";

uint16_t device_id = 0xFF45;

#define SLAVE_ADDR 0x68

uint8_t get_len_of_data(void)
{
  return (uint8_t)strlen(name_msg);
}
void setup() {
  // Define the LED pin as Output
  pinMode (LED, OUTPUT);
  
  // Start the I2C Bus as Slave on address 9
  Wire.begin(SLAVE_ADDR); 
  
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);


}


//write
void receiveEvent(int bytes) {
  active_command = Wire.read();    // read one character from the I2C 
}

//read
void requestEvent() {

  if(active_command == 0X51)
  {
    uint8_t len = get_len_of_data();
    Wire.write(&len,1);
    active_command = 0xff;
  }
  

  if(active_command == 0x52)
  {
   // Wire.write(strlen(name));
    Wire.write(name_msg,get_len_of_data());
   // Wire.write((uint8_t*)&name_msg[32],18);
    active_command = 0xff;
  }
  //Wire.write("hello "); // respond with message of 6 bytes
  // as expected by master
}
void loop() {
  

}
