#include "LX16A.h"
#include <SoftwareSerial.h>

//Set up a serial port to communicate with Pi:
SoftwareSerial poofBrainSerialPort(10, 11); // RX, TX
//Create an object for the LX16A motor controller library:

LX16A LX16A_controller;

//Define Servo Id's (Each LX-16A servo has an id set manually via LobotSerialServoSetId()):
//TODO: Separate these into Shoulder, Elbow, Hands and Legs classes and initialize object with servo Id's
#define servoUpward_id 2
#define servoFrontward_id 3
#define servoDownward_id 4
#define servoBackward_id 5
#define servoHeadRotate_id 6
#define servoHeadUpDown_id 7
#define servoElbowRotate_id 8
#define servoElbowUpDown_id 9

//Map the range of motion of the servo, based on manual callibration:
int tolerance = 0; //Degrees slack off of tendons
int upwardMap (int value) {
  return map(value,0,1000,100,1000)+tolerance;
}
int downwardMap (int value) {
  return map(value,0,1000,1000,0)-tolerance;
}
int backwardMap (int value) {
  return map(value,0,1000,0,1000)-tolerance;
}
int frontwardMap (int value) {
  return map(value,0,1000,0,1000) + tolerance;
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////Movement Functions////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int speed = 2000;
void moveUpandDown(int targetPosition){
  LX16A_controller.LobotSerialServoMove(servoUpward_id,upwardMap(targetPosition), speed);
  delay(200);
  LX16A_controller.LobotSerialServoMove(servoDownward_id,downwardMap(targetPosition), speed);
  delay(200);
}
void moveFrontandBack(int targetPosition){
  LX16A_controller.LobotSerialServoMove(servoFrontward_id,frontwardMap(targetPosition), speed);
  delay(200);
  LX16A_controller.LobotSerialServoMove(servoBackward_id,backwardMap(targetPosition), speed);
  delay(200);
}
unsigned long ElbowWaveTime;
boolean elbowWaving = false;
void elbowWave(){
  Serial.println("Servo position");
  Serial.println(LX16A_controller.LobotSerialServoReadPosition(servoElbowUpDown_id));
  if (elbowWaving && LX16A_controller.LobotSerialServoReadPosition(servoElbowUpDown_id) >=790){
  LX16A_controller.LobotSerialServoMove(servoElbowUpDown_id,300, speed+1500);
  }
  else if (elbowWaving&&LX16A_controller.LobotSerialServoReadPosition(servoElbowUpDown_id) <=310){
  LX16A_controller.LobotSerialServoMove(servoElbowUpDown_id,800, speed+1500);
  }
  else if (!elbowWaving){
    elbowWaving = true;
    LX16A_controller.LobotSerialServoMove(servoElbowUpDown_id,800, speed);
  }
}
void moveToXY(int frontBack, int upDown){ //Frontback = x, updown = y
  moveUpandDown(upDown);
  moveFrontandBack(frontBack);
  }

////////////////////////////////////////////////////////////////////////////////
///////////////////////Arduino Functions////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  LX16A_controller.setSerial(&Serial);
  poofBrainSerialPort.begin(9600);
  delay(1000);
}

int positionx = 1000;
int positiony = 000;
int increment = -100;
void loop() {  
  elbowWave();
  moveToXY(positionx,positiony);

//
//  for (int x = 0; x<256;x++)
//    LX16A_controller.LobotSerialServoSetID(x , 9);
//  positionx = positionx+increment;
//  positiony = positiony+increment;
//  if (positionx<0)
//  increment = 100;
//  else if (positionx>1000)
//  increment = -100;
//  
  //Serial communication between the brain (Pi) and duino:
  if (poofBrainSerialPort.available()) {
    //positiony = poofBrainSerialPort.read();
    //positiony = poofBrainSerialPort.read();
    //Serial.write("Read a number ");
    //Serial.write(poofBrainSerialPort.read());
  }
  if (Serial.available()) {
    //poofBrainSerialPort.write(Serial.read());
  } 
}
