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
int stopped = 0;

//Commands
static const byte REQUEST = 1;
static const byte RECEIVED = 2;
static const byte P_ANGLE = 3;
static const byte STOP = 4;
float lastAngle = -180;

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);
unsigned int pingSpeed = 100;
unsigned long pingTimer;
float last1 = 0;
float last2 = 0;

void setup(){
  this_node_address = (EEPROM.read(0) << 8) | EEPROM.read(1); // Radio address for this node
  other_node_address = 0x69ff;
  Serial.begin(57600);
  radio.begin();
  setupListening();
  pingTimer = millis();
  motor1.attach(5);
  motor2.attach(6);
  motor1.write(0);
  motor2.write(0);
}

void printOnPC(const byte type, float number){
//  if(type == P_ANGLE)
//    Serial.println(number);
//  uint16_t pc = 0x009f;  
//  radio.stopListening();  
//  radio.openWritingPipe(pc);
//  struct payload myPayload = {type, number};
//  radio.write(&myPayload, sizeof(payload));
//  setupListening();
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
float scaleAndFloor(int speedNum){
  //scale from 0 - 254 to 0 - 20
  //map won't work, it returns long
  float num = speedNum;
  if(speedNum > 254)  
    num = speedNum;
  else if(speedNum < 0)
    num = 0;
  return ((num*20)/255);
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
//  Serial.println("write to controller");
}

void requestPing(){
  writeToController(REQUEST);
  printOnPC(REQUEST, 0);
}

void handleInstruction(struct payload * instruction){
  printOnPC(RECEIVED, 0);
//  if(instruction->command == STOP){
//    stopRobot();
//    stopped = 1;
//  }
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
  if(!stopped){
    if (millis() >= pingTimer) {
      float dist1 = 0;
      float dist2 = 0;
      pingTimer += pingSpeed;
      requestPing();
      int p1 = sonar1.ping();
  //    if(p1 == 0){
  ////      dist1 = last1;
  //    }else{
  //      float temp = microsToInches(p1); // Send ping, get ping time in microseconds (uS).
  //      if(abs(temp - last1) >  15)
  //        dist1 = last1;
  //      else
  //        dist1 = temp;
  //    }
      dist1 = microsToInches(p1);
      delay(50); //Witout this, it gives false values
      requestPing();
      int p2 = sonar2.ping();
  //    if(p2 == 0){
  //      dist2 = last2;
  //    }else{
  //      float temp = microsToInches(p2); // Send ping, get ping time in microseconds (uS).
  //      if(abs(temp - last2) >  15)
  //        dist2 = last2;
  //      else
  //        dist2 = temp;
  //    }
      dist2 = microsToInches(p2);
//      if(dist1 != 0 && dist2 != 0)
        pid(dist1, dist2);
    }
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
  float KDTurn = 0; 
  float angle = calcAngle(dist1, dist2);
  float forward = KP * nDist;
  Serial.println(nDist);
  printOnPC(P_ANGLE, angle);
  if(((int)angle != 0) && ((int)angle != 90)){
//    printOnPC(P_ANGLE, angle);
    float KDTerm = 0;
    if(lastAngle == -180)
      KDTerm = KDTurn*(angle - lastAngle);
    int turnSpeed = KPTurn * angle + KDTerm;// * forward;
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
    lastAngle = angle;
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

