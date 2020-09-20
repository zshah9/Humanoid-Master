// Author: Zachary Shah, Aug 2019

// This program moves a 6 inch linear actuator forward and backward via pushbutton control, and the extension of the linear
// actuator from resting position in inches can be read in the Serial Monitor at any given time.

const int yellow = 9;     //
const int green = 8;   // 

void setup() {

  pinMode(yellow, OUTPUT); 
  pinMode(green, OUTPUT); 

  Serial.begin(9600);
  
}

void loop() {

  digitalWrite(yellow, HIGH);
  digitalWrite(green, LOW);

  delay(1000);

  digitalWrite(yellow, LOW);
  digitalWrite(green, HIGH);

  delay(1000);
}
