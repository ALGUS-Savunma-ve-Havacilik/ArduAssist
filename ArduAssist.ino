#include "SoftwareServo.h"
#include "HobbyRadioReceiver.h"


// Specify the number of channels,
//   followed by the pins the channels are attached to


HobbyRadioReceiver rec( 6, A0, A1, A2, A3, A4, A5);

SoftwareServo * servoArray;
int * pins;

int iNumServos = 6;
int iThrottle = 2;

unsigned long previousMillis    = 0;
unsigned long previousMilli2    = 0;

int val;

void setup()
{
  servoArray = new SoftwareServo[6];
  pins = new int[3,5,6,9,10,11];
  pinMode(13,OUTPUT);
  for (int i =0; i< iNumServos; i++)
  {
    servoArray[i].attach(pins[i]);
    if (i == iThrottle)
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
  Serial.println(rec.getNumChannels());  
}

void loop()
{
  unsigned long currentMillis = millis();

  // here is where you'd put code that needs to be running all the time.
  if(currentMillis - previousMilli2 > 50)
  {
    previousMilli2 = currentMillis;
    for (int i = 1; i <= rec.getNumChannels(); i++ )
    {
      val = rec.check(i);
      Serial.print( val );
      if (i < rec.getNumChannels())
      {
        Serial.print( "\t" );
      }
      Serial.println( "\n\n\n\n\n\n\n\n\n\n\n" );
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