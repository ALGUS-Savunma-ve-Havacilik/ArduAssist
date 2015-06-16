
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

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "SoftwareServo.h"
#include "HobbyRadioReceiver.h"
#include "ServoMod.h"

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

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
  
  Serial.begin(9600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  
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
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
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

