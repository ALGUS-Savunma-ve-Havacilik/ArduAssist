//Servo setup is as follows
// Receiver
//  Ch1 - Right stick vertical -	pitch
//  Ch2 - Right stick horizontal -	roll
//  Ch3 - Left stick vertical -	throttle - not routed to Arduino.
//  Ch4 - Left stick horizontal -	yaw
//  Ch5 - right knob -			flight variables
//  Ch6 - left knob -			flaps/spoiler adjust
//  left switch -				Radio Setting - DUAL RATE
// right switch -				Radio Setting - THROTTLE CUT

//Arduino PWM out

//  PWM1 - elevator
//  PWM2 - rudder
//  PWM3 - left aileron
//  PWM4 - right aileron
//  PWM5 - unknown/unused
//  PWM6 - unknown/unused

//If flaps/spoilers are active, they should get at most 30% of available throw.  If the knob is at 50%, ailerons get 100% of throw.


#include "SoftwareServo.h"
#include "HobbyRadioReceiver.h"
#include "ServoMod.h"

enum output_pins // enum to get the pin number we are after from the pins[] array
{
  elevator,
  rudder,
  leftAileron,
  rightAileron,
  throttleOut
};

enum input_pins
{
  pitch = 1, //Because the HobbyRadioReceiver indexes from 1 for input pins, we start from 1
  roll,
  yaw,
  throttleIn,
  scale,
  flapSpoil
};

enum max_inputs
{
  max_pos = 0,
  max_neg
};

SoftwareServo * servoArray;
int * pins;
HobbyRadioReceiver * rec;

int iNumServos = 6;
int iThrottle = 2;
int ledPin = 13; 
bool ledOn = false;

unsigned long previousMillis    = 0;
unsigned long previousMilli2    = 0;
unsigned long calibrationMillis = 0;

ServoMod mod;

int val;


void setup()
{
  servoArray = new SoftwareServo[6];
  pins = new int[3,5,6,9,10,11];
  
  // Specify the number of channels,
  //   followed by the pins the channels are attached to
  rec = new HobbyRadioReceiver( 6, A0, A1, A2, A3, A4, A5);
  pinMode(ledPin,OUTPUT);
  for (int i =0; i< iNumServos; i++)
  {
    servoArray[i].attach(pins[i]);
    if (i == throttleOut)
    {
      servoArray[i].write(0);
    }
    else
    {
      servoArray[i].write(90);
    }
  }
  
  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  
  Serial.print( "Num Channels: " );
  Serial.println(rec->getNumChannels());
}

void loop()
{
  unsigned long currentMillis = millis();
  
  while (millis() < 10000) // first ten seconds for calibration.  Get max travel for all six raw channels coming from the transmitter
  {
    unsigned long curCalibrationMillis = millis();
    if (curCalibrationMillis - calibrationMillis > 250)
    {
      if (ledOn)
      {
        digitalWrite(ledPin,LOW);
        ledOn = false;
      }
      else
      {
        digitalWrite(ledPin,HIGH);
        ledOn = true;
      }
    }
    for (int i = 1; i <= rec->getNumChannels(); i++ )
    {
      int j = i-1; //bloody indexing of the servo library - index starts at 1, not 0
      int curVal = rec->check(i);
      if (curVal < mod.servos[j].getMaxNeg())
      {
        mod.servos[j].setMaxNeg(curVal);
      }
      if (curVal > mod.servos[j].getMaxPos())
      {
        mod.servos[j].setMaxPos(curVal);
      }
    }
  }

  // here is where you'd put code that needs to be running all the time.
  if(currentMillis - previousMilli2 > 50)
  {
    if (ledOn)
    {
      digitalWrite(ledPin,LOW);
      ledOn = false;
    }
    else
    {
      digitalWrite(ledPin,HIGH);
      ledOn = true;
    }
    previousMilli2 = currentMillis;
    for (int i = 1; i <= rec->getNumChannels(); i++ )
    {
      val = rec->check(i);
      Serial.print( val );
      if (i < rec->getNumChannels())
      {
        Serial.print( "\t" );
      }
      Serial.println( "\n\n\n\n\n\n\n\n\n\n\n" );
      
      if (i == roll)
      {
        servoArray[leftAileron].write(mod.iLeftAileronSimple(val));
        servoArray[rightAileron].write(mod.iRightAileronSimple(val));
        
        int iLeft;
        int iRight;
        mod.AileronComplex(val,&iLeft,&iRight,90);
        servoArray[leftAileron].write(iLeft);
        servoArray[rightAileron].write(iRight);
      }
      else
      {
        servoArray[i-1].write(mod.noChange(val));
      }
    }
  }

  // Servo(s) refresh only every 20 msec
  if(currentMillis - previousMillis > 20)
  {
    previousMillis = currentMillis;
    for (int i =0; i< iNumServos; i++)
    {
      servoArray[i].refresh();
    }
  }
}

