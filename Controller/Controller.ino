#include <EEPROM.h>
#include <SPI.h>
#include <string.h>
//#include "nRF24L01.h"
#include "RF24.h"

#define ultraPin 6
#define ledPin 4
#define MAX_DISTANCE 200

struct payload{
  byte command;
  float number;
};

int led = 1;
uint16_t other_node_address;
uint16_t this_node_address;
RF24 radio(9,10);
//Commands
static const byte REQUEST = 1;
static const byte RECEIVED = 2;
static const byte P_ANGLE = 3;


void setup()
{
  pinMode(ledPin, OUTPUT);
  this_node_address = (EEPROM.read(0) << 8) | EEPROM.read(1); // Radio address for this node
  other_node_address = 0xdc10;
  Serial.begin(57600);
  radio.begin();
  setupListening();
  Serial.println("Start");
  pinMode(ultraPin, OUTPUT);
}

void setupListening(){
  radio.openReadingPipe(1, this_node_address);
  radio.startListening();
}

void handleMessage(struct payload * message){
  switch(message->command){
    case REQUEST:
      struct payload command;
      command.command = RECEIVED;
      command.number = 0;
      sendPulse();
      sendMessage(command);
      break;
  }
}

void sendPulse(){
  digitalWrite(ledPin, led);
  led = !led;
  pinMode(ultraPin, OUTPUT);
  digitalWrite(ultraPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultraPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(ultraPin, LOW);
}

void sendMessage(struct payload command){
  radio.stopListening();  
  radio.openWritingPipe(other_node_address);
  radio.write(&command, sizeof(payload));
  setupListening();
}

void loop(){
  //Check if new instruction received
  if(radio.available()){
    //Read instruction if available
    Serial.println("message received");
    struct payload * current_payload = (struct payload *) malloc(sizeof(struct payload));
    radio.read( current_payload, sizeof(struct payload) );
    handleMessage(current_payload);
    free(current_payload);
  }
}

