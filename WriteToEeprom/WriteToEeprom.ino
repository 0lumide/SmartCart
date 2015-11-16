#include <EEPROM.h>

uint16_t this_node_address = -1;//0xdc10;
void setup() {
  Serial.begin(57600);
  // put your setup code here, to run once:
  int firstByte = this_node_address >> 8;
  int secondByte = 0x00ff & this_node_address;
  
  if(this_node_address != -1){
    //Do the actual writing here
    EEPROM.write(0, firstByte);
    EEPROM.write(1, secondByte);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  
  uint16_t address = (EEPROM.read(0) << 8) | EEPROM.read(1);
  Serial.print("This address is: ");
  Serial.println(address);
  delay(100); 
}
