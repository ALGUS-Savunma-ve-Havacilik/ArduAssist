#include <HobbyRadioReceiver.h> // see https://github.com/ScottCProjects/Arduino/blob/master/Libraries/HobbyRadioReceiver/HobbyRadioReceiver.h
#include <Adafruit_SoftServo.h>

/*
* Literal.ino
*
* Created: 7/6/2015 3:54:03 PM
* Author: Richard Smith
*/

// only working with one channel and one servo right now.  Will eventually scale to six.
//HobbyRadioReceiver rec( 2, A0, A1);
//Adafruit_SoftServo servo1, servo2;
HobbyRadioReceiver rec( 1, A1);
Adafruit_SoftServo servo1;

int ledPin = 13;          //Pin number for on-board LED.  Probably a #define somewhere for this.
bool servoInit = false;   // Have we finished the servo initialization?
bool toggle = false;      // Toggle for led on pin 13
int count = 0;            // interrupt counter
int val1Last = 90;        // Last value for servo1 from loop()
int val2last = 90;        // Last value for servo2 from loop()
int rx1MinMax[2] = {0,0}; // min and max values coming off radio receiver
int skipCount = 0;

// =============================================
// ===               50ms SERVO REFRESH ROUTINE                ===
// =============================================

//Timer2 Overflow Interrupt, called every 1ms
ISR(TIMER2_OVF_vect) {
  if (servoInit)
  {
    count++;               //Increments the interrupt counter
    if(count > 50){
      toggle = !toggle;    //toggles the LED state
      count = 0;           //Resets the interrupt counter
      digitalWrite(ledPin,toggle);
      servo1.write(val1Last);
      //Serial.print("ch1: ");
      Serial.println(val1Last);
      servo1.refresh();
      
      //Serial.print("\t\tch1Min: ");
      //Serial.print(rx1MinMax[0]);
      //Serial.print("\tmax: ");
      //Serial.println(rx1MinMax[1]);
      
      //servo2.write(val2last);
      //Serial.print("\tch2: ");
      //Serial.println(val2last);
      //servo2.refresh();
    }
  }
  TCNT2 = 130;           //Reset Timer to 130 out of 255
  TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};

void setup()
{
  
  Serial.begin(115200);
  
  servo1.attach(3);
  servo1.write(90);
  servo1.refresh();

  //servo2.attach(5);
  //servo2.write(90);
  //servo2.refresh();
  
  servoInit = true;
  rec.setMode(1,"center");
  
  //Setup Timer2 to fire every 1ms
  TCCR2B = 0x00;        //Disbale Timer2 while we set it up
  TCNT2  = 130;         //Reset Timer Count to 130 out of 255
  TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;        //Timer2 Control Reg A: Wave Gen Mode normal
  TCCR2B = 0x05;        //Timer2 Control Reg B: Timer Prescaler set to 128

}

void loop()
{

  int val1 = rec.checkServo(1);
  //Serial.println(val1);
  if (val1 < rx1MinMax[0] && val1 >=0)
  {
    rx1MinMax[0] = val1;
  }
  if (val1 > rx1MinMax[1])
  {
    rx1MinMax[1] = val1;
  }
  //Below line commented as we will stick to raw values for now.
  int val1Translate = constrain(map(val1,17,154,0,179),0,180);
  //int val1Translate = constrain(map(val1,-255,255,0,179),0,180);
  //int val1Translate = val1;
  
  // do not write to the servo, that will be done in the interrupt loop.
  if (val1Last < val1Translate - 5 || val1Last > val1Translate + 5)
  {
    skipCount++;
  }    
  else
  {
    val1Last = val1Translate;    
    skipCount = 0;
  }
  
  if (skipCount > 3)
  {
    skipCount = 0;
    val1Last = val1Translate;    
  }
  
}
