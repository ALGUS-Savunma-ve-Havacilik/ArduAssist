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

SoftwareServo * servoArray;
int * pins;
HobbyRadioReceiver * rec;

int iNumServos = 6;
int iThrottle = 2;

unsigned long previousMillis    = 0;
unsigned long previousMilli2    = 0;

ServoMod mod;

int val;


void setup()
{
	servoArray = new SoftwareServo[6];
	pins = new int[3,5,6,9,10,11];
 
 // Specify the number of channels,
 //   followed by the pins the channels are attached to
 rec = new HobbyRadioReceiver( 6, A0, A1, A2, A3, A4, A5);
	pinMode(13,OUTPUT);
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

	// here is where you'd put code that needs to be running all the time.
	if(currentMillis - previousMilli2 > 50)
	{
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

