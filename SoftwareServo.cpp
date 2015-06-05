//****************************************************************************
// @Module        Arduino-compatible Servo Library implementation.
// @Filename      SoftwareServo.cpp
// @Project       Arduino_XMC
//----------------------------------------------------------------------------
// @Controller    Infineon XMC1x00
//
// @Compiler      GCC
//
// @Codegenerator DAvE 3.0 Compatible
//
// @Description   This file contains low level functions to implement the
//					Servo Library.
//
// @Link          http://playground.arduino.cc/ComponentLib/Servo
//
//----------------------------------------------------------------------------
// @Date          21/04/2014 01.17.23
//
//****************************************************************************


//****************************************************************************
// @Project Includes
//****************************************************************************

// #include Arduino HEADER FILES.
#include "Arduino.h"
#include "SoftwareServo.h"

SoftwareServo::SoftwareServo(void) {
  isAttached = false;
  servoPin = 255;
  angle = 90;
}

void SoftwareServo::attach(uint8_t pin) {
  servoPin = pin;
  angle = 90;
  isAttached = true;
  pinMode(servoPin, OUTPUT);
}

void SoftwareServo::detach(void) {
  isAttached = false;
  pinMode(servoPin, INPUT);
}

boolean SoftwareServo::attached(void) {
  return isAttached;
}

void SoftwareServo::write(uint8_t a) {
  angle = a;

  if (! isAttached) return;
  micros = map(a, 0, 180, 1000, 2000);
}

void SoftwareServo::refresh(void) {
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(micros);
  digitalWrite(servoPin, LOW);
}

//****************************************************************************
// 							       END OF FILE
//****************************************************************************