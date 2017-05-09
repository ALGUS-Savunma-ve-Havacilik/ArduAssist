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

//#define SERVOMIN 160 // this is the 'minimum' 60hz duration : 0 degrees+ 
//#define SERVOMAX 580 // this is the 'maximum' 60hz duration : 180 degrees
//#define SERVOMID 370 // This is the 'middle' 60hz duration : 90 degrees

#define MINPULSE 1140 // Minimum pulse from receiver
#define MIDPULSE 1450 // Centered pulse from receiver
#define MAXPULSE 1850 // maximum pulse from receiver

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

class servo
{
  public:
  uint16_t start = 160;
  uint16_t midpt = 580;
  uint16_t endpt = 370;
};

servo servos[15];

//uint8_t * chans;
uint8_t readPins[] = {A0, A1, A2, A3 } ;
uint8_t aileronPins[] = {5, 6} ; // Put aileron servos individually on servo board 4 & 5 9ndex from 0

void setup()
{

  uint8_t readSize =  sizeof(readPins) / sizeof(uint8_t);

  servos[aileronPins[0] ].start = 265;
  servos[aileronPins[0] ].midpt = 370;
  servos[aileronPins[0] ].endpt = 475;

  servos[aileronPins[1] ].start = 265;
  servos[aileronPins[1] ].midpt = 370;
  servos[aileronPins[1] ].endpt = 475;
  
  pwm.begin();
  Serial.begin(9600);
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  for (uint8_t count = readSize; count >=0 ; count--)
  {
    pinMode(readPins[count], INPUT);
    
    if (count == 1)
    {
      pwm.setPWM(aileronPins[0], 0, servos[aileronPins[0]].midpt );
      pwm.setPWM(aileronPins[1], 0, servos[aileronPins[1]].midpt);
    }
    else
    {
      pwm.setPWM(count, 0, servos[count].midpt);
    }
  }  
}

void loop()
{
  uint8_t readSize =  sizeof(readPins) / sizeof(uint8_t);
  for (uint8_t count = 0; count < sizeof(readPins) / sizeof(uint8_t); count++)
  {
	  unsigned long duration = pulseIn(readPins[count], HIGH);
    Serial.print(readPins[count]);
    Serial.print(F(","));
    Serial.println(duration);
    //int angle = map(duration,MINPULSE,MAXPULSE,-90,90);
    
    //pulselength = constrain(map(degrees, 0, 180, SERVOMIN, SERVOMAX),SERVOMIN,SERVOMAX); // gets a pulse length for an angle
    //unsigned long mappedDur = constrain(map(duration, MINPULSE, MAXPULSE, SERVOMIN, SERVOMAX),SERVOMIN,SERVOMAX); 
    uint16_t mappedDur = 1500;
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
