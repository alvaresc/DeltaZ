#include <Servo.h>
#include "Arduino.h"
#include "DeltaRobot.h"

Delta::Delta() {
  // Constructor for the delta class. This function allows you to create an instance of the delta class and sets some properties of the robot.
  e = 25;     // end effector
  f = 50;     // base
  re = 60.0;
  rf = 30.0;
  sqrt3 = sqrt(3.0);
  pi = 3.141592653;    // PI
  sin120 = sqrt3 / 2.0;
  cos120 = -0.5;
  tan60 = sqrt3;
  sin30 = 0.5;
  tan30 = 1.0 / sqrt3;
  rMax = 30;
  zMin = -75;
  zMax = -35;
}

void Delta::setupMotors(int pin1, int pin2, int pin3) {
  // Initialize the robot's serovs. This must be done in Arduino's setup funtion
  servo1.attach(pin1);
  servo2.attach(pin2);
  servo3.attach(pin3);
}

void Delta::goHome() {
  /* Delta will go to its home position (x,y,z) = (0,0,-47.06) where all joint angles are zero. The robot will report its joint angles to the serial monitor*/
  goToAngle(0,0,0);
  reportAngles();
}

void Delta::goTo(float x, float y, float z) {
  /* Inputs the target position, (x,y,z). First, this function tests if the target position is in the workspace. 
   If so, it calculates the inverse kineamtics, moves the robot to that position. It not, the robot reports that it isnt in the workspace.
   Finally, the robot reports its current joint angles.*/
  int inWorkspace = testInWorkspace(x, y, z);

  if (inWorkspace){
    x0 = x;
    y0 = y;
    z0 = z;
    delta_calcInverse(x0, y0, z0, t1, t2, t3);
    servo1.write(-t1 + 90);
    servo2.write(-t2 + 90);
    servo3.write(-t3 + 90);  
  } else {
    Serial.println("NOT IN WORKSPACE");
  }
  reportAngles();
}

int Delta::testInWorkspace(float x, float y, float z) {
  /* inputs target x,y, and z position. Tests if the position if the position is in the current worksapce of the robot (set by rMax, zMin, and zMax.
  If the point is in the workspace, return 1, else return 0*/
  float r = sqrt(x * x + y * y);
//  Serial.println(String(" test") + round(x) + String(" ") + round(z) + String(" ") + round(z));
  if ((r <= rMax) && (z <= zMax) && (z >= zMin)) {
    return 1;
  } else {
    return 0;
  }
}

void Delta::reportPosition(){
  /* Report the curent position of the robot*/
  Serial.print("ATPOS ");
  Serial.print(x0,2);
  Serial.print(" ");
  Serial.print(y0,2);
  Serial.print(" ");
  Serial.println(z0,2);
}

void Delta::reportAngles(){
  /* Report the current position of the robot*/
  Serial.println(String("ATANG ") + round(t1) + String(" ") + round(t2) + String(" ") + round(t3));
}

void Delta::goToAngle(int angle1, int angle2, int angle3) {
  /* inputs angles angle1-3. Claculates forward kineamtics and tests if position is in the workspace. If so, the robot goes to those angles. 
   *  If not, the root reports that it is not in the workspace. Then, report the position of the robot. */
  float x0Old = x0;
  float y0Old = y0;
  float z0Old = z0;

  delta_calcForward(angle1, angle2, angle3, x0, y0, z0);
  /* Visit https://hypertriangle.com/~alex/delta-robot-tutorial/ for more information*/
  int inWorkspace = testInWorkspace(x0, y0, z0);
  if (inWorkspace){
    t1 = angle1;
    t2 = angle2;
    t3 = angle3;
    servo1.write(-t1 + 90);
    servo2.write(-t2 + 90);
    servo3.write(-t3 + 90);
  } else {
    Serial.println("NOT IN WORKSPACE");
    x0 = x0Old;
    y0 = y0Old;
    z0 = z0Old;
  }
  reportPosition();
  
}

int Delta::delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
  /* Visit https://hypertriangle.com/~alex/delta-robot-tutorial/ for more information*/
  float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
  //float y1 = yy1;
  y0 -= 0.5 * 0.57735    * e;    // shift center to edge
  // z = a + b*y
  float a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2 * z0);
  float b = (y1 - y0) / z0;
  // discriminant
  float d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
  if (d < 0) return -1; // non-existing point
  float yj = (y1 - a * b - sqrt(d)) / (b * b + 1); // choosing outer point
  float zj = a + b * yj;
  theta = 180.0 * atan(-zj / (y1 - yj)) / pi + ((yj > y1) ? 180.0 : 0.0);
  if ((theta < -180) || (theta > 180))
    return -1;
  return 0;
}

int Delta::delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
  /* Visit https://hypertriangle.com/~alex/delta-robot-tutorial/ for more information*/
  theta1 = theta2 = theta3 = 0;
  int stat1 = delta_calcAngleYZ(x0, y0, z0, theta1);
  int stat2 = delta_calcAngleYZ(x0 * cos120 + y0 * sin120, y0 * cos120 - x0 * sin120, z0, theta2); // rotate coords to +120 deg
  int stat3 = delta_calcAngleYZ(x0 * cos120 - y0 * sin120, y0 * cos120 + x0 * sin120, z0, theta3); // rotate coords to -120 deg
  return stat1 + stat2 + stat3;
}

int Delta::delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
  /* Visit https://hypertriangle.com/~alex/delta-robot-tutorial/ for more information*/
  float t = (f - e) * tan30 / 2;
  float dtr = pi / (float)180.0;

  theta1 *= dtr;
  theta2 *= dtr;
  theta3 *= dtr;

  float y1 = -(t + rf * cos(theta1));
  float z1 = -rf * sin(theta1);

  float y2 = (t + rf * cos(theta2)) * sin30;
  float x2 = y2 * tan60;
  float z2 = -rf * sin(theta2);

  float y3 = (t + rf * cos(theta3)) * sin30;
  float x3 = -y3 * tan60;
  float z3 = -rf * sin(theta3);

  float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

  float w1 = y1 * y1 + z1 * z1;
  float w2 = x2 * x2 + y2 * y2 + z2 * z2;
  float w3 = x3 * x3 + y3 * y3 + z3 * z3;

  // x = (a1*z + b1)/dnm
  float a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

  // y = (a2*z + b2)/dnm;
  float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

  // a*z^2 + b*z + c = 0
  float a = a1 * a1 + a2 * a2 + dnm * dnm;
  float b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
  float c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re);

  // discriminant
  float d = b * b - (float)4.0 * a * c;
  if (d < 0) return -1; // non-existing point
  
  z0 = -(float)0.5 * (b + sqrt(d)) / a;
  x0 = (a1 * z0 + b1) / dnm;
  y0 = (a2 * z0 + b2) / dnm;
  return 0;
}
