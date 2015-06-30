

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

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <MPU6050.h>
#include <helper_3dmath.h>
#include <Adafruit_SoftServo.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "HobbyRadioReceiver.h"
#include "ServoMod.h"
#include "ArduAssist.h"

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

Adafruit_SoftServo * servoArray;
int * pins;
HobbyRadioReceiver * rec;

int iNumServos = 6;
int iThrottle = 2;
int ledPin = 13;
bool ledOn = false;
bool servoInit = false;

unsigned long previousMillis    = 0;
unsigned long previousMilli2    = 0;
unsigned long calibrationMillis = 0;
unsigned int toggle = 0;  //used to keep the state of the LED
unsigned int count = 0;   //used to keep count of how many interrupts were fired

ServoMod mod;

int val;

int iRollMapped = 90;
int iPitchMapped = 90;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

// ================================================================
// ===               50ms SERVO REFRESH ROUTINE                ===
// ================================================================

//Timer2 Overflow Interrupt Vector, called every 1ms
ISR(TIMER2_OVF_vect) {
  if (servoInit)
  {
    count++;               //Increments the interrupt counter
    if(count > 20){
      toggle = !toggle;    //toggles the LED state
      count = 0;           //Resets the interrupt counter
      digitalWrite(ledPin,toggle);
      for (int i=0; i< 6; i++)
      {
        servoArray[i].refresh();
        //Serial.print(i);
        //Serial.print(":");
        //Serial.print(written[i]);
      }
      //Serial.println("---done---");
    }
  }
  TCNT2 = 130;           //Reset Timer to 130 out of 255
  TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
};

// ================================================================
// ===                                                   Setup                                                        ===
// ================================================================

void setup()
{
  pins = new int[3,5,6,9,10,11];
  
  Serial.begin(115200);
  //while (!Serial); // wait for Leonardo enumeration, others continue immediately
  // Specify the number of channels,
  //   followed by the pins the channels are attached to
  rec = new HobbyRadioReceiver( 6, A0, A1, A2, A3, A6, A7);
  servoArray = new Adafruit_SoftServo[rec->getNumChannels()];
  
  
  pinMode(ledPin,OUTPUT);
  for (int i =0; i< iNumServos; i++)
  {
    mod.servos[i].setMaxTravel(180);
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
  
  servoInit = true;
  
  //Setup Timer2 to fire every 1ms
  TCCR2B = 0x00;        //Disbale Timer2 while we set it up
  TCNT2  = 130;         //Reset Timer Count to 130 out of 255
  TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;        //Timer2 Control Reg A: Wave Gen Mode normal
  TCCR2B = 0x05;        //Timer2 Control Reg B: Timer Prescaler set to 128
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //Uncomment for calibration to horizontal
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-791);
  mpu.setYAccelOffset(-3500);
  mpu.setZAccelOffset(1494); // 1688 factory default for my test chip
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-13);
  mpu.setZGyroOffset(23);
  
  
  //mpu.setXAccelOffset(-2680);
  //mpu.setYAccelOffset(-3483);
  //mpu.setZAccelOffset(3107); // 1688 factory default for my test chip
  //mpu.setXGyroOffset(14);
  //mpu.setYGyroOffset(-16);
  //mpu.setZGyroOffset(26);


  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    //set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Init failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
  Serial.print( "Num Channels: " );
  Serial.println(rec->getNumChannels());
  
  Serial.println( "Starting calibration");
  
  for (int i = 1; i <= rec->getNumChannels(); i++ )
  {
    int j = i-1; //bloody indexing of the servo library - index starts at 1, not 0
    int curVal = -255;
    if (curVal < mod.servos[j].getMaxNeg())
    {
      mod.servos[j].setMaxNeg(curVal);
    }
    if (curVal > mod.servos[j].getMaxPos())
    {
      mod.servos[j].setMaxPos(curVal);
    }
  }
  
  unsigned long calibrationStart = millis();
  while (millis() - calibrationStart < 10000) // first ten seconds for calibration.  Get max travel for all six raw channels coming from the transmitter
  {
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
  Serial.println( "Ending calibration");
  for (int i = 0; i< rec->getNumChannels(); i++)
  {
    Serial.print("Max neg ch " );
    Serial.print(i+1);
    Serial.print(" : ");
    Serial.print(mod.servos[i].getMaxNeg());
    Serial.println("");
    Serial.print("Max pos ch " );
    Serial.print(i+1);
    Serial.print(" : ");
    Serial.print(mod.servos[i].getMaxPos());
    Serial.println("");
  }
  
  Serial.println(millis());
  delay(5000);
}

// ================================================================
// ===                                       Main program loop                                               ===
// ================================================================

void loop()
{
  
  //wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
   {
    // other program behavior stuff here
    for (int i = 1; i <= rec->getNumChannels(); i++ )
    {
      val = rec->check(i);
      servoArray[i-1].write(mod.noChange(val,i-1));
    }
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    float fRoll = ypr[2] * 180/M_PI;
    float fPitch = ypr[1] * 180/M_PI;
    
    iRollMapped = (int)constrain( map(fRoll,-90,90,0,180),0,180);
    iPitchMapped = (int)constrain( map(fPitch,-90,90,0,180),0,180);
  }  
}