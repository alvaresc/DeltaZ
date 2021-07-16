#include "Arduino.h"
#include <Servo.h>

class Delta
{
  public:
    Delta();
    Servo servo1;
    Servo servo2;
    Servo servo3;
    void goHome();
    void goTo(float x, float y, float z);
    void goToAngle(int angle1, int angle2, int angle3);
    int delta_calcAngleYZ(float x0, float y0, float z0, float &theta);
    int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3);
    int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0);
    void reportPosition();
    void reportAngles();
    float t1;
    float t2;
    float t3;
    float x0;
    float y0;
    float z0;
    float rMax;
    float zMin;
    float zMax;
    int testInWorkspace(float x, float y, float z);
    void setupMotors(int pin1,int pin2, int pin3);

  private:
    float e;     // end effector
    float f;     // base
    float re;
    float rf;
    float sqrt3;
    float pi;    // PI
    float sin120;
    float cos120;
    float tan60;
    float sin30;
    float tan30;       
};
