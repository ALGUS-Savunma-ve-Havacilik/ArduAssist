#include "HobbyRadioReceiver.h" // see https://github.com/ScottCProjects/Arduino/blob/master/Libraries/HobbyRadioReceiver/HobbyRadioReceiver.h
//#include <Adafruit_SoftServo.h>

/*
* Literal.ino
*
* Created: 7/6/2015 3:54:03 PM
* Author: Richard Smith
*/

#define STEP 8
/*
unsigned long previousMillis    = 0;
unsigned long previousMilli2    = 0;
bool bUP                        = true;
int pos                         = 0;
*/

HobbyRadioReceiver rec( 5, A0, A1, A2, A3, A4, A5);
/*
Adafruit_SoftServo servo1;

int ledPin = 13;          // Pin number for on-board LED.  Probably a #define somewhere for this.
bool servoInit = false;   // Have we finished the servo initialization?
bool toggle = false;      // Toggle for led on pin 13
int count = 0;            // interrupt counter
int val1Last = 90;        // Last value for servo1 from loop()
int val2last = 90;        // Last value for servo2 from loop()
int rx1MinMax[2] = {0,0}; // min and max values coming off radio receiver
int skipCount = 0;

const int numReadings = 5;

int readings[numReadings];      // the readings from the analog input
int index = 0;                  // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

// ==================================
// === 20ms SERVO REFRESH ROUTINE ===
// ==================================

//Timer2 Overflow Interrupt, called every 1ms
ISR(TIMER2_OVF_vect) {
  if (servoInit)
  {
    count++;               //Increments the interrupt counter
    if(count > 20){
      toggle = !toggle;    //toggles the LED state
      count = 0;           //Resets the interrupt counter
      digitalWrite(ledPin,toggle);
      
      if (pos < 0)
      {
        pos = 0;
      }
      if (pos > 180)
      {
        pos = 180;
      }
      servo1.write(pos);
      //Serial.print("ch1: ");
      servo1.refresh();
    }
  }
  TCNT2 = 130;           //Reset Timer to 130 out of 255
  TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};
*/
void setup()
{
  
  Serial.begin(115200);
  
  //servo1.attach(3);
  //servo1.write(90);
  //servo1.refresh();

  //servo2.attach(5);
  //servo2.write(90);
  //servo2.refresh();
  
  //servoInit = true;
  rec.setMode(1,"center");
  rec.setMode(2,"center");
  rec.setMode(3,"center");
  rec.setMode(4,"center");
  rec.setMode(5,"center");

  /*
  //Setup Timer2 to fire every 1ms
  TCCR2B = 0x00;        //Disbale Timer2 while we set it up
  TCNT2  = 130;         //Reset Timer Count to 130 out of 255
  TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;        //Timer2 Control Reg A: Wave Gen Mode normal
  TCCR2B = 0x05;        //Timer2 Control Reg B: Timer Prescaler set to 128
  
  //for (int thisReading = 0; thisReading < numReadings; thisReading++)
  //readings[thisReading] = 0;
  */

}

void loop()
{  
  Serial.print(rec.check(1));
  Serial.print(",");
  Serial.print(rec.check(2));
  Serial.print(",");
  Serial.print(rec.check(3));
  Serial.print(",");
  Serial.print(rec.check(4));
  Serial.print(",");
  Serial.println(rec.check(5));
  delay(100); 
}
