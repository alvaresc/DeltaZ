#include "DeltaRobot.h"
#include <Servo.h>
#include <math.h>
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
Delta myDelta = Delta();
float distance = 0;
float height = -35.0;

//int analogVal;
//int analogPin = A0;
//int zVal;
//int xVal;
//int yVal;
//int analogX;
//int analogY;
//int analogZ;
//int durabilityCount = 0;
//int zPin = A0;
//int xPin = A3;
//int yPin = A4;


//  trigger echo distance(cm)
//int maxReading = 20;
//NewPing sonar(7,6,maxReading);

void setup(){
  Serial.begin(9600);
  inputString.reserve(200);
  myDelta.setupMotors(9,10,11);
  myDelta.goHome();

}

void loop(){
//  int durDelay = 200;
//  myDelta.goTo(0,0,-35);
//  delay(durDelay);
//  myDelta.goTo(0,0,-75);
//  delay(durDelay);
//  myDelta.goHome();
//  delay(durDelay);
//  myDelta.goTo(30,0,-47.06);
//  delay(durDelay);
//  myDelta.goTo(0,30,-47.06);
//  delay(durDelay);
//  myDelta.goTo(-30,0,-47.06);
//  delay(durDelay);
//  myDelta.goTo(0,-30,-47.06);
//  delay(durDelay);
//  durabilityCount = durabilityCount+1;
//  Serial.println("Durability " + String(durabilityCount));
  

//  distance = avg5US();
//  height = 2*distance-75.0;
//  Serial.print("distance: ");
//  Serial.println(distance);
//  myDelta.goTo(0,0,height);

//    height = IRSensor.getDistance();

   
//    analogX = avgAnalogFast(xPin);
//    xVal = map(analogX, 0, 1022, -30, 30);
//    analogY = avgAnalogFast(yPin);
//    yVal = map(analogY, 0, 1022, -30, 30);
//    analogZ = avgAnalogFast(zPin);
//    zVal = map(analogZ, 0, 1022, -75, -35);
    
//    height = map(analogVal, 0, 1022, -75, -35);
    
//    Serial.println("Height: "+String(analogVal));
//    delay(1000);
//    delay(1000);
//    delay(1000);
//    myDelta.goTo(xVal,yVal,zVal);
//delay(1000);
    
   

  
  if (stringComplete) {
    inputString.toUpperCase();
    inputString.trim(); // Gets rid of the carriage response ASCII 13
//    Serial.println(inputString); //option send to MATLAB
    if (inputString.equalsIgnoreCase("HOME")){
      myDelta.goHome();
    } 

    if (inputString.startsWith("GOTO")){ // parameterized messages
      String posString = inputString.substring(5);
      int commaIndex = posString.indexOf(',');
      //  Search for the next comma just after the first
      int secondCommaIndex = posString.indexOf(',', commaIndex + 1);
      String firstValue = posString.substring(0, commaIndex);
      String secondValue = posString.substring(commaIndex + 1, secondCommaIndex);
      String thirdValue = posString.substring(secondCommaIndex + 1); // To the end of the string
      float x = firstValue.toInt();
      float y = secondValue.toInt();
      float z = thirdValue.toInt();
      myDelta.goTo(x,y,z);   
    }

    if (inputString.startsWith("ANG")){ // parameterized messages
      String posString = inputString.substring(4);
      int commaIndex = posString.indexOf(',');
      //  Search for the next comma just after the first
      int secondCommaIndex = posString.indexOf(',', commaIndex + 1);
      String firstValue = posString.substring(0, commaIndex);
      String secondValue = posString.substring(commaIndex + 1, secondCommaIndex);
      String thirdValue = posString.substring(secondCommaIndex + 1); // To the end of the string
      int ang1 = firstValue.toInt();
      int ang2 = secondValue.toInt();
      int ang3 = thirdValue.toInt();
      myDelta.goToAngle(ang1,ang2,ang3);   
    }
    if (inputString.startsWith("SF")){ // parameterized messages
      int SFNum = inputString.substring(3).toInt();
      Serial.println(SFNum);
      if (SFNum == 1){
        SF1();    
      } else if (SFNum == 2){
        SF2(); 
      }else if (SFNum == 3){
        SF3(); 
      }else if (SFNum == 4){
        SF4(); 
      }
    }
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}
void goCircle(int r,float z){
  float x;
  float y;

  for (int i = 0; i<360; i=i+2){
    x = r*cos(i*3.14159/180);
    y = r*sin(i*3.14159/180);
    myDelta.reportPosition();
    myDelta.goTo(x,y,z);
    
    delay(1);
    
  }
  myDelta.goHome();
}

//float avg5US(){
//  float sumReadings = 0;
//  int reading = 0;
//  for (int i = 0; i<5; i++){
//    reading = sonar.ping_cm();
//    if (reading<1){
//      reading = maxReading;
//    }
//    sumReadings = sumReadings + reading;
//    delay(10);
//  }
//  return sumReadings/5;
//}

//float avg5Analog(int analogPin){
//  float sumReadings = 0;
//  int reading = 0;
//  for (int i = 0; i<5; i++){
//    reading = analogRead(analogPin);
//    if (reading<1){
//      reading = maxReading;
//    }
//    sumReadings = sumReadings + reading;
//    delay(10);
//  }
//  return sumReadings/5;
//}

//float avgAnalogFast(int analogPin){
//  float sumReadings = 0;
//  int reading = 0;
//  for (int i = 0; i<5; i++){
//    reading = analogRead(analogPin);
//    if (reading<1){
//      reading = maxReading;
//    }
//    sumReadings = sumReadings + reading;
////    delay(10);
//  }
//  return sumReadings/5;
//}

void SF1(){
//  ADD YOUR OWN COOL FUNCTION :)
Serial.println("SF1");
goCircle(25,-70.5);

}

void SF2(){
//  ADD YOUR OWN COOL FUNCTION :)
Serial.println("SF2");
}

void SF3(){
//  ADD YOUR OWN COOL FUNCTION :)
Serial.println("SF3");
}

void SF4(){
//  ADD YOUR OWN COOL FUNCTION :)
Serial.println("SF4");
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') { // \n is line feed ASCII 10
      stringComplete = true;
    } else{
      // add it to the inputString:
      inputString += inChar;
    }
  }
}
