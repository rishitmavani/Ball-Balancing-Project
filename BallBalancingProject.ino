
/////////Ball and Plate///////////////////////////////
/*
  BALL AND PLATE PID CONTROL
*/
//////////////////////////////////////////////////////
///Libraries///
#include <PID_v1.h>
#include <stdint.h>
#include <TouchScreen.h>
#include <Servo.h>
// Definitions TOUCH PINS
#define YP A0 //0
#define XM A1 //1
#define YM 3  //3
#define XP 4  //4
// For better pressure precision, we need to know the resistance
// between XP and XM Use any multimeter to read it
// For the 10.2" its 505 ohms across the X plate
#define Rx 600 //  Widerstandswert in Ohm vom Touchscreen in X-Richtung gemessen  
TouchScreen ts = TouchScreen(XP, YP, XM, YM, Rx);

// Koordinaten Touchscreen
TSPoint p;
double xmin = 48.0;
double xmax = 935.0;
double   xm = (xmax - xmin) / 2.0; // x - Mitte
double xLength = 304.1; // Breite vom Touchscreen in mm bei 10.1"
double ymin = 95.0;
double ymax = 935.0;
double   ym = (ymax - ymin) / 2.0; // y - Mitte
double yLength = 228.1; // Höhe vom Touchscreen in mm bei 10.1"
double convertX = xLength / (xmax - xmin);  // converts raw x values to mm. found through manual calibration
double convertY = yLength / (ymax - ymin);   // converts raw y values to mm. found through manual calibration

double x0 =  0;
double y0 =  0;
double x1 =  70;
double y1 =  47;
double x2 = -70;
double y2 = 47;
double x3 = -70;
double y3 =  -47;
double x4 =  70;
double y4 =  -47;
int state = 0;
long StartTime = 0;
long duration = 20 * 1000; // Zeitdauer für eien Position

// servos variables
Servo servoX; //X axis
Servo servoY; //Y axis

/////TIME SAMPLE
int Ts = 50;
unsigned long Stable = 0;
//PID const

double Kpx = 0.101;
double Kix = 0.08;
double Kdx = 0.12;

double Kpy = 0.3;
double Kiy = 0.08;
double Kdy = 0.12;


double deadX = 1;
double deadY = 1;
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
  servoX.write(OutputX); //ServoX in waagerechte Servo-Position X
  servoY.write(OutputY); //ServoY in waagerechte Servo-Position Y

  //INIT OF TOUSCHSCREEN
  p = ts.getPoint();

  //INIT SETPOINT, Mittelpunkt der Platte
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
  StartTime = millis();
}

void loop()
{

  Serial.println(state);
  // OutputX=90;
  // OutputY=90;
  // servoX.write(OutputX); //ServoX in waagerechte Servo-Position X
  // servoY.write(OutputY); //ServoY in waagerechte Servo-Position Y

  while (Stable < 150) //REGULATION LOOP
  {
    p = ts.getPoint();   //measure pressure on plate
    if (p.x > 0 ) //ball is on plate
    {
      servoX.attach(5); //connect servos
      servoY.attach(6);
      setDesiredPosition();
      noTouchCount = 0;
      p = ts.getPoint(); // measure actual position
      InputX = (p.x - xmin - xm) * convertX;  // read and convert X coordinate
      InputY = (p.y - ymin - ym) * convertY; // read and convert Y coordinate
      if ((InputX > SetpointX - deadX && InputX < SetpointX + deadX && InputY > SetpointY - deadY && InputY < SetpointY + deadY)) //if ball is close to setpoint
      {
        // Stable = Stable + 1; //increment STABLE
        digitalWrite(13, HIGH);
      }
      else
      {
        digitalWrite(13, LOW);
      }
      myPIDX.Compute();  // action control X compute
      myPIDY.Compute();  // action control  Y compute
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
     servoX.write(OutputX);  //control
     servoY.write(OutputY);  //control
    //Serial.print(SetpointX);   Serial.print(",");  Serial.print(SetpointY);  Serial.print(",");  Serial.print(InputX);Serial.print(","); Serial.println(InputY);

  } // END OF REGULATION LOOP

  servoX.detach();//detach servos
  servoY.detach();

  ///KONTROL STABILITY////
  while (Stable == 125) //if is stable
  { //still measure actual postiion
    setDesiredPosition();
    p = ts.getPoint();
    InputX = (p.x - xmin - xm) * convertX;  // read and convert X coordinate
    InputY = (p.y - ymin - ym) * convertY; // read and convert Y coordinate
    if (InputX < SetpointX - deadX || InputX > SetpointX + deadX || InputY > SetpointY + deadY || InputY < SetpointY - deadY  ) //if ball isnt close to setpoint
    {
      servoX.attach(5); //again attach servos
      servoY.attach(6);
      digitalWrite(13, LOW);
      Stable = 0; //change STABLE state
    }

  }//end of STABLE LOOP
}//loop end

////////////////////////Functions//////////////////
///// DESIRED POSITION
void setDesiredPosition()
{
  switch (state)
  { case 0:  SetpointX = x0;
      SetpointY = y0;
      if (millis() - StartTime > duration) {
        state = 1;
        StartTime = millis();
      };
      break;
    case 1:
      SetpointX = x1;
      SetpointY = y1;
      if (millis() - StartTime > duration) {
        state = 2;
        StartTime = millis();
      };
      break;
    case 2:  SetpointX = x2;
      SetpointY = y2;
      if ( millis() - StartTime > duration) {
        state = 3;
        StartTime = millis();
      };
      break;
    case 3:  SetpointX = x3;
      SetpointY = y3;
      if ( millis() - StartTime > duration) {
        state = 4;
        StartTime = millis();
      };
      break;
    case 4:  SetpointX = x4;
      SetpointY = y4;
      if ( millis() - StartTime > duration) {
        state = 1;
        StartTime = millis();
      };
      break;
    default : StartTime = millis();
      state = 1;
      break;
  }
}
