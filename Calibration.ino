/*
Touchscreen test + calibartion
both Servos in 90 degree position
*/
///Libraries///
#include <stdint.h>
#include "TouchScreen.h"
#include<Servo.h>
// Definitions TOUCH PINS
#define YP A0 //0
#define XM A1 //1
#define YM 3 //3
#define XP 4 //4
// For better pressure precision, we need to know the resistance
// between XP and XM use any multimeter to read it
// For the 15" its 600 ohms across the X plate
#define Rx 600 // Widerstandswert in Ohm vom Touchscreen in X-Richtung gemessen
TouchScreen ts = TouchScreen(XP, YP, XM, YM, Rx);
// Koordinaten Touchscreen
TSPoint p;
// servos variables
Servo servoX; //X axis
Servo servoY; //Y axis
void setup()
{
Serial.begin(115200);
servoX.attach(5);
servoY.attach(6);
servoX.write(90); //Servo X in horizontal position
servoY.write(90); //Servo Y in horizontal position
delay(100);
}
void loop()
{
p = ts.getPoint(); // measure actual position
if (p.x > 0 ){ // ball is on plate
Serial.print("x= "); Serial.print(p.x); Serial.print(" ");
Serial.print("y= "); Serial.println(p.y);
}
delay(100);
} //loop end
