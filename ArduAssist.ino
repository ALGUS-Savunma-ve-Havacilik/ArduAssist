//Servo setup is as follows
// Receiver
//  Ch1 - Right stick vertical -	pitch on A0
//  Ch2 - Right stick horizontal -	roll on A1
//  Ch3 - Left stick vertical -	throttle - not routed to Arduino.
//  Ch4 - Left stick horizontal -	yaw on A2
//  Ch5 - right knob -			flight variables on A6
//  Ch6 - left knob -			flaps/spoiler adjust on A3
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


#define SERVOMIN 160 // this is the 'minimum' 60hz duration : 0 degrees
#define SERVOMAX 580 // this is the 'maximum' 60hz duration : 180 degrees
#define SERVOMID 370 // This is the 'middle' 60hz duration : 90 degrees

#define MINPULSE 1250 // Minimum pulse from receiver
#define MIDPULSE 1450 // Centered pulse from receiver
#define MAXPULSE 1650 // maximum pulse from receiver

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//uint8_t * chans;
uint8_t readPins[] = {A0, A1, A2, A3, A6};
uint8_t aileronPins[] = {4,5}; // Put aileron servos individually on servo board 4 & 5 index from 0

void setup()
{
  for (int i = 0; i<sizeof(readPins) / sizeof(uint8_t); i++)
  {
    pinMode(readPins[i], INPUT);
  }
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  for (uint8_t count = 0; count < sizeof(readPins) / sizeof(uint8_t); count++)
  {
    if (count == 1)
    {
      pwm.setPWM(aileronPins[0], 0, SERVOMID);
      pwm.setPWM(aileronPins[1], 0, SERVOMID);
    }
    else
    {
      pwm.setPWM(count, 0, SERVOMID);
    }
  }  
}

void loop()
{
  for (uint8_t count = 0; count < sizeof(readPins) / sizeof(uint8_t); count++)
  {
	  unsigned long duration = pulseIn(readPins[count], HIGH);

    int angle = map(duration,MINPULSE,MAXPULSE,-90,90);
    
    //pulselength = constrain(map(degrees, 0, 180, SERVOMIN, SERVOMAX),SERVOMIN,SERVOMAX); // gets a pulse length for an angle
    unsigned long mappedDur = constrain(map(duration, MINPULSE, MAXPULSE, SERVOMIN, SERVOMAX),SERVOMIN,SERVOMAX); 
    if (count == 1)
    {
      pwm.setPWM(aileronPins[0], 0, mappedDur);
      pwm.setPWM(aileronPins[1], 0, mappedDur);
    }
    else
    {
      pwm.setPWM(count, 0, mappedDur);
    }
  }
}
