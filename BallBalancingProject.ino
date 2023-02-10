///Libraries///
#include <PID_v1.h>
#include <stdint.h>
#include <TouchScreen.h>
#include <Servo.h>
#include <IRremote.h>
// Definitions TOUCH PINS
#define YP A0 //0
#define XM A1 //1
#define YM 3 //3
#define XP 4 //4
const byte IR_RECEIVE_PIN = 7;
#define Rx 600 // resistance to sense the touch
TouchScreen ts = TouchScreen(XP, YP, XM, YM, Rx);
// coordination of Touchscreenscreen with help of screentouch calibration code
TSPoint p;
double xmin = 48.0;
double xmax = 935.0;
double xm = (xmax - xmin) / 2.0; // x - Mitte
double xLength = 304.1; // Breite vom Touchscreen in mm bei 10.1"
double ymin = 95.0;
double ymax = 935.0;
double ym = (ymax - ymin) / 2.0; // y - Mitte
double yLength = 228.1; // Höhe vom Touchscreen in mm bei 10.1"
double convertX = xLength / (xmax - xmin); // converts raw x values to mm. found through manual calibration
double convertY = yLength / (ymax - ymin); // converts raw y values to mm. found through manual calibration
//point for making square
double x0 = 0;
double y0 = 0;
double x1 = 70;
double y1 = 47;
double x2 = -70;
double y2 = 47;
double x3 = -70;
double y3 = -47;
double x4 = 70;
double y4 = -47;
int state = 0;
// servos variables
Servo servoX; //X axis
Servo servoY; //Y axis
/////TIME SAMPLE
int Ts = 50; // to check for a point in this time sample
unsigned long Stable = 0; //variable to check for stability
// the PID variable for both motors
double Kpx = 0.15;
double Kix = 0.10;
double Kdx = 0.10;
//double Kpy = 0.3;
double Kpy = 0.15;
double Kiy = 0.08;
double Kdy = 0.10;
double deadX = 5;
double deadY = 5;
double xin = 0.0;
double yin = 0.0;
//INIT PID
double SetpointX, InputX, OutputX; //for X
double SetpointY, InputY, OutputY; //for Y
PID myPIDX(&InputX, &OutputX, &SetpointX, Kpx, Kix, Kdx, DIRECT);
PID myPIDY(&InputY, &OutputY, &SetpointY, Kpy, Kiy, Kdy, DIRECT);
unsigned int noTouchCount = 0; //viariable for noTouch
void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); //LED INIT
  servoX.attach(5);
  servoY.attach(6);
  // Make plate flat
  OutputX = 90;
  OutputY = 90;
  servoX.write(OutputX);
  servoY.write(OutputY);
  //INIT OF TOUSCHSCREEN
  p = ts.getPoint();
  //INIT SETPOINT and Mid point
  SetpointX = 0.0;
  SetpointY = 0.0;
  //Setup PID Controller
  myPIDX.SetMode(AUTOMATIC);
  myPIDX.SetOutputLimits(40, 100);
  myPIDY.SetMode(AUTOMATIC);
  myPIDY.SetOutputLimits(40, 100);
  // TIME SAMPLE
  myPIDX.SetSampleTime(Ts);
  myPIDY.SetSampleTime(Ts);
  delay(100);
  // StartTime = millis();
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}
void loop()
{
  while (Stable < 150) //REGULATION LOOP
  {
    p = ts.getPoint(); //measure pressure on plate
    if (p.x > 0 ) //ball is on plate
    {
      //connect servos
      servoX.attach(5);
      servoY.attach(6);
    // function to check for irreceiver
    if (IrReceiver.decode())
    {
      int varstate = IrReceiver.decodedIRData.command;
      // set for state depending on the button pressed
    if(varstate == 12){
      state= 1;
    }
    else if(varstate == 24 ){
      state = 2;
    }
    else if(varstate == 94){
      state = 3;
    }
    else if(varstate == 8){
      state = 4;
    }
    else if(varstate == 28){
      state = 0;
    }
    setDesiredPositionWithRemote();
    IrReceiver.resume();
    }
    Serial.println(state);
    noTouchCount = 0;
    p = ts.getPoint(); // measure actual position
    InputX = (p.x - xmin - xm) * convertX; // read and convert X coordinate
    InputY = (p.y - ymin - ym) * convertY; // read and convert Y coordinate
    if ((InputX > SetpointX - deadX && InputX < SetpointX + deadX && InputY > SetpointY - deadY && InputY < SetpointY + deadY)) //if ball is close to setpoint
    {
      Stable = Stable + 1; //increment STABLE
    }
    myPIDX.Compute(); // action control X compute
    myPIDY.Compute(); // action control Y compute
    }
    else //if there is no ball on plate
    {
      noTouchCount++; //increment no touch count
    if (noTouchCount == 75)
    {
      noTouchCount++;
      OutputX = 90; //make plate flat
      OutputY = 90;
      servoX.write(OutputX);
      servoY.write(OutputY);
    }
    if (noTouchCount == 150) //if there is no ball on plate longer
      {
        servoX.detach(); //detach servos
        servoY.detach();
      }
    }
    servoX.write(OutputX); //control
    servoY.write(OutputY); //control
    } // END OF REGULATION LOOP
    servoX.detach();//detach servos
    servoY.detach();
    ///KONTROL STABILITY////
    while (Stable == 150) //if is stable
    { //still measure actual postiion
      // setDesiredPosition();
      setDesiredPositionWithRemote();
      p = ts.getPoint();
      InputX = (p.x - xmin - xm) * convertX; // read and convert X coordinate
      InputY = (p.y - ymin - ym) * convertY; // read and convert Y coordinate
      Serial.println(InputX );
      Serial.println(InputY);
      if (InputX < SetpointX - deadX || InputX > SetpointX + deadX || InputY > SetpointY + deadY || InputY < SetpointY - deadY ) //if ball isnt close to setpoint
      {
        servoX.attach(5); //again attach servos
        servoY.attach(6);
        digitalWrite(13, LOW);
        Stable = 0; //change STABLE state
      }
    }//end of STABLE LOOP
}//loop end
////////////////////////Functions//////////////////
//ir remote function
void setDesiredPositionWithRemote()
{
  switch (state)
    { 
      case 0:
        SetpointX = x0;
        SetpointY = y0;
        break;
      case 1:
        SetpointX = x1;
        SetpointY = y1;
        break;
      case 2:
        SetpointX = x2;
        SetpointY = y2;
        break;
      case 3:
        SetpointX = x3;
        SetpointY = y3;
        break;
      case 4:
        SetpointX = x4;
        SetpointY = y4;
        break;  
      default :
        state = 1;
        break;
    }
}
