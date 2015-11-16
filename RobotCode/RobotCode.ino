#include <NewPing.h>
#include <EEPROM.h>
#include <SPI.h>
#include <string.h>
#include <math.h>
#include "RF24.h"
#include <Servo.h>
#include <vexMotor.h>

vexMotor motor1;
vexMotor motor2;

struct payload{
  byte command;
  float number;
};

//Configurable Section
#define TRIGGER_PIN1  7
#define ECHO_PIN1     8
#define TRIGGER_PIN2  A4
#define ECHO_PIN2     A5
#define MAX_DISTANCE 200

//End configurable section

uint16_t other_node_address;
uint16_t this_node_address;
RF24 radio(9,10);

//Commands
static const byte REQUEST = 1;
static const byte RECEIVED = 2;
static const byte P_ANGLE = 3;
static const byte STOP = 4;

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);

unsigned int pingSpeed = 100;
unsigned long pingTimer;

void setup(){
  this_node_address = 0xdc10;
  other_node_address = 0x69ff;
  Serial.begin(57600);
  radio.begin();
  pingTimer = millis();
  motor1.attach(5);
  motor2.attach(6);
  setupListening();
  stopRobot();
}

void setupListening(){
  radio.openReadingPipe(1, this_node_address);
  radio.startListening();
}

void stopRobot(){
  //Make robot stop
  motor1.write(0);
  motor2.write(0);
  Serial.println("stopRobot");
}

int clip(int speedNum, int limit){
  if (speedNum < -limit)
    return -limit;
  if (speedNum > limit)
    return limit;
  return speedNum;
}

void writeToController(const byte message){
  radio.stopListening();  
  radio.openWritingPipe(other_node_address);
  struct payload myPayload = {message, 0};
  radio.write(&myPayload, sizeof(payload));
  setupListening();
  Serial.println("write to controller");
}

void requestPing(){
  writeToController(REQUEST);
  Serial.println("request");
}

void handleInstruction(struct payload * instruction){
  Serial.println("received");
}

void loop(){
  //Check if new instruction received
  if(radio.available()){
    //Read instruction if available
    struct payload * current_payload = (struct payload *) malloc(sizeof(struct payload));
    radio.read( current_payload, sizeof(struct payload) );
    handleInstruction(current_payload);
    free(current_payload);
  }
  if (millis() >= pingTimer) {
    float dist1 = 0;
    float dist2 = 0;
    pingTimer += pingSpeed;
    requestPing();
    int p1 = sonar1.ping();
    dist1 = microsToInches(p1);
    delay(50); //Witout this, it gives false values
    requestPing();
    int p2 = sonar2.ping();
    dist2 = microsToInches(p2);
    Serial.print(dist1);
    Serial.print(" ");
    Serial.println(dist2);
//    pid(dist1, dist2);
  }
}

void pid(float dist1, float dist2){
  float nDist = 0.5*(dist1+dist2);
  Serial.print(dist1);
  Serial.print(" ");
  Serial.print(dist2);
  Serial.print(" ");
  Serial.println(nDist);
  float KP = 1.3;
  float KPTurn = 2.3;
  float angle = calcAngle(dist1, dist2);
  float forward = KP * nDist;
  Serial.println(nDist);
  int turnSpeed = KPTurn * angle;// + KDTerm;// * forward;
  if(((abs(angle) < 5)) && nDist > 18){
    motor1.write(clip(forward, 80));
    motor2.write(clip(-forward, 80));
  }else if((abs(angle) > 10) && (nDist > 18)){
    motor1.write(clip(turnSpeed, 80));
    motor2.write(clip(turnSpeed, 80));
  }else{
    motor1.write(0);
    motor2.write(0);
  }
}

float microsToInches(int micro){
//  if(micro == 0)
//    return 0;
  return 0.0138*micro + 6.4551;
}

float calcAngle(float dist1, float dist2){
  double a = 10.75;//distance between two receivers in inches
  double num = sq(dist2) - sq(dist1);
  double sqrtden = 2*sq(dist2) + 2*sq(dist1) - sq(a);
  double den = a*sqrt(sqrtden);
  double angle = acos(num/den);
  return (90 - (angle * 57.2958));
}

