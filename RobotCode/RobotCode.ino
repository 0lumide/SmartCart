#include <NewPing.h>
#include <EEPROM.h>
#include <SPI.h>
#include <string.h>
#include <math.h>
#include "RF24.h"
#include <Servo.h>
#include <vexMotor.h>
#include "IR.h"

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
#define MAX_DISTANCE 100

#define IR1  A0
#define IR2  A1
#define IR3  A2
#define IR4  A3

#define Encoder1PinA  1
#define Encoder1PinB  2
#define Encoder2PinA  4
#define Encoder2PinB  3
#define WHEEL_TICKS 50
//End configurable section

uint16_t other_node_address;
uint16_t this_node_address;
RF24 radio(9,10);
volatile unsigned int Encoder1Pos = 0;
volatile unsigned int Encoder2Pos = 0;

//Commands
static const byte REQUEST = 1;
static const byte RECEIVED = 2;
static const byte P_ANGLE = 3;
static const byte STOP = 4;
static const byte START = 5;

float lastAngle = 0;
int stopped = 0;

IR ir1(IR1, 200, 160);
IR ir2(IR2, 200, 160);
IR ir3(IR3, 300, 20);
IR ir4(IR4, 300, 20);

NewPing sonar1(TRIGGER_PIN1, ECHO_PIN1, MAX_DISTANCE);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE);

unsigned int pingSpeed = 100;
unsigned long pingTimer;
unsigned long lastValid;

void setup(){
  this_node_address = 0xdc10;
  other_node_address = 0x69ff;
  Serial.begin(57600);
  radio.begin();
  pingTimer = millis();
  lastValid = pingTimer;
  motor1.attach(5);
  motor2.attach(6);
  setupListening();
  stopRobot();
//  attachInterrupt(digitalPinToInterrupt(Encoder1PinB), EncoderEvent1, HIGH);
//  attachInterrupt(digitalPinToInterrupt(Encoder2PinB), EncoderEvent2, CHANGE);
}

void setupListening(){
  radio.openReadingPipe(1, this_node_address);
  radio.startListening();
}

void stopRobot(){
  //Make robot stop
  motor1.write(0);
  motor2.write(0);
}

void timeout(int dist1, int dist2){
  if(dist1 && dist2)
    lastValid = millis();
  else if((millis() - lastValid) > 30000){
    stopped = 1;
  }
}

bool coastIsClear(){
  return !ir1.isBlocked() 
    && !ir2.isBlocked()
    && !ir3.isBlocked()
    && !ir4.isBlocked();
}

bool robotIsStuck(){
  return (ir1.isBlocked() && ir3.isBlocked())
    || (ir1.isBlocked() && ir4.isBlocked())
    || (ir2.isBlocked() && ir3.isBlocked())
    || (ir2.isBlocked() && ir4.isBlocked());
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
}

void requestPing(){
  writeToController(REQUEST);
}

void handleInstruction(){
    //Read instruction if available
    struct payload * current_payload = (struct payload *) malloc(sizeof(struct payload));
    radio.read( current_payload, sizeof(struct payload) );
    switch(current_payload->command){
      case START: stopped = 0;
        timeout(1, 1);
    }
    free(current_payload);
}

void loop(){
  //Check if new instruction received
  if(radio.available()){
    handleInstruction();
  }
  if ((millis() >= pingTimer) && !stopped) {
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

    //checks if the robot should timeout
    timeout(dist1, dist2);
    
    if(coastIsClear()){
      pid(dist1, dist2);
    }else if(dist1 && dist2){
      if(robotIsStuck()){
        stopRobot();
      }else if(ir3.isBlocked() || ir4.isBlocked()){
        motor1.write(60);
        motor2.write(-60);
      }else if(ir1.isBlocked()&&!ir2.isBlocked()){
        motor1.write(-80);
        motor2.write(-40);
      }else if(!ir1.isBlocked()&&ir2.isBlocked()){
        motor1.write(40);
        motor2.write(80);
      }
    }
  }
}

void pid(float dist1, float dist2){
  float nDist = 0.5*(dist1+dist2);
  float KP = 1.3;
  float KPTurn = 3;
  float KDTurn = 1.8;
  float angle = calcAngle(dist1, dist2);
  float forward = KP * nDist;
  int turnSpeed = KPTurn * angle - KDTurn * (lastAngle - angle);
  if(((abs(angle) < 5)) && (nDist > 30)){
    motor1.write(clip(forward, 80));
    motor2.write(clip(-forward, 80));
  }else if((abs(angle) > 10) && (nDist > 30)){
    motor1.write(clip(turnSpeed + 0.8*forward, 80));
    motor2.write(clip(turnSpeed - 0.8*forward, 80));
  }else if(nDist <= 30){
    stopRobot();
  }
  lastAngle = angle;
}

float microsToInches(int micro){
  if(micro == 0)
    return 0;
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

// Encoder event for the interrupt call
void EncoderEvent1(){
  // Read for data and bit changes
  // This is gray-code logic
  if (digitalRead(Encoder1PinA) == HIGH){
    if (digitalRead(Encoder1PinB) == LOW)
      Encoder1Pos++;
    else
      Encoder1Pos--;
  }else{ 
    if (digitalRead(Encoder1PinB) == LOW)
      Encoder1Pos--;
    else
      Encoder1Pos++;
  }
}

// Encoder event for the interrupt call
void EncoderEvent2(){
  // Read for data and bit changes
  // This is gray-code logic
  if (digitalRead(Encoder2PinA) == HIGH){
    if (digitalRead(Encoder2PinB) == LOW)
      Encoder2Pos++;
    else
      Encoder2Pos--;
  }else{ 
    if (digitalRead(Encoder2PinB) == LOW)
      Encoder2Pos--;
    else
      Encoder2Pos++;
  }
}

