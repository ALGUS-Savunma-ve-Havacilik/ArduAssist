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

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//from http://www.servodatabase.com/servo/towerpro/sg90
//They seem quite stable but as mentioned by another robotics user they won't do 180 degrees. 
//Mine went from about 4 degrees (Using setting 581) to 176 degrees (Using 2380). 
//They will hit most angles in between fairly accurately though.

//I worked out that the centre (90 degrees) was 1480 and calculated 
//45 and 135 and it went right to them. Pity about upper and lower (0 and 180) not being available though.
#define SERVOMIN 160 // this is the 'minimum' pulse length in ms
#define SERVOMAX 580 // this is the 'maximum' pulse length in ms
#define SERVOMID 370

#define MINPULSE 1250 // Minimum pulse from receiver
#define MAXPULSE 1650 // maximum pulse from receiver

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//uint8_t * chans;
uint8_t * readPins;

void setup()
{
  //chans = new uint8_t[0,1,3,4,5];
  readPins = new uint8_t[A0,A1,A2 ];//,A1,A2,A3,A6];
  for (int i = 0; i<sizeof(readPins) / sizeof(uint8_t); i++)
  {
    pinMode(readPins[i], INPUT);
  }
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  for (uint8_t count = 0; count < sizeof(readPins) / sizeof(uint8_t); count++)
  {
     pwm.setPWM(count, 0, SERVOMID);
  }  
}

void loop()
{
  for (uint8_t count = 0; count < sizeof(readPins) / sizeof(uint8_t); count++)
  {
	//unsigned long duration = pulseIn(readPins[count], HIGH);
	//int iDegrees = (int)constrain( map(fRoll,-90,90,0,180),0,180);
  //pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
  //unsigned long mappedDur = map(duration, MINPULSE, MAXPULSE, SERVOMIN, SERVOMAX); 
    //pwm.setPWM(count, 0, mappedDur);
    pwm.setPWM(count, 0, SERVOMID);
  }
}
