#include <Servo.h> // include servo library
// Create servo objects
Servo servo1;  
Servo servo2;
Servo servo3;


void setup() {
 // Tell arduino which servos are connected to which pmw pins
 servo1.attach(9);
 servo2.attach(10);
 servo3.attach(11);
 // Set the servos to 90 degrees
 servo1.write(90);
 servo2.write(90);
 servo3.write(90);
}

void loop() {
  // do nothing
}
