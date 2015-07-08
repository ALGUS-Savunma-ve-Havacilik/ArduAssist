#include <HobbyRadioReceiver.h>
#include <Adafruit_SoftServo.h>

/*
 * Literal.ino
 *
 * Created: 7/6/2015 3:54:03 PM
 * Author: richards
 */ 


HobbyRadioReceiver rec( 2, A0, A1);
Adafruit_SoftServo servo1, servo2;


int iNumServos = 2;
int ledPin = 13;
bool servoInit = false;
bool toggle = false;
int count = 0;
int val1Last = 90;
int val2last = 90;
int servo1MinMax[2] = {0,0};

// ================================================================
// ===               50ms SERVO REFRESH ROUTINE                ===
// ================================================================

//Timer2 Overflow Interrupt Vector, called every 1ms
ISR(TIMER2_OVF_vect) {
  if (servoInit)
  {
    count++;               //Increments the interrupt counter
    if(count > 50){
      toggle = !toggle;    //toggles the LED state
      count = 0;           //Resets the interrupt counter
      digitalWrite(ledPin,toggle);
      servo1.write(val1Last);
      Serial.print("ch1: ");
      Serial.print(val1Last);
      servo1.refresh();
      
      Serial.print("\tch1Min: ");
      Serial.print(servo1MinMax[0]);
      Serial.print("\tmax: ");
      Serial.println(servo1MinMax[1]);
      
      //servo2.write(val2last);
      //Serial.print("\tch2: ");
      //Serial.println(val2last);
      servo2.refresh();
    }
  }
  TCNT2 = 130;           //Reset Timer to 130 out of 255
  TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};

void setup()
{
  
  Serial.begin(115200);
   //while (!Serial); // wait for Leonardo enumeration, others continue immediately
   // Specify the number of channels,
   //   followed by the pins the channels are attached to
   
servo1.attach(3);
servo1.write(90);
servo1.refresh();

servo2.attach(5);
servo2.write(90);
servo2.refresh();
   
   servoInit = true;
   
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

      int val1 = rec.check(1);
      if (val1 < servo1MinMax[0])
      {
        servo1MinMax[0] = val1;
      }
      if (val1 > servo1MinMax[1])
      {
        servo1MinMax[1] = val1;
      }
      //int val1Translate = constrain(map(val1,servo1MinMax[0],servo1MinMax[1],0,179),0,180);
      int val1Translate = constrain(map(val1,-255,255,0,179),0,180);
      val1Last = val1Translate;
      //servo1.write(valAvg);
      

	  /* add main program code here, this code starts again each time it ends */

}
