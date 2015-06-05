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


#ifndef SOFTWARESERVO_H_
#define SOFTWARESERVO_H_
#ifdef __cplusplus

#include "Arduino.h"

class SoftwareServo
{
public:
	SoftwareServo(void);
	void attach(uint8_t pin);
	void detach();
	boolean attached();
	void write(uint8_t a);
	void refresh(void);
private:
 boolean isAttached;
 uint8_t servoPin, angle;
 uint16_t micros;
};

#endif  // __cplusplus
#endif /* SOFTWARESERVO_H_ */
//****************************************************************************
// 							       END OF FILE
//****************************************************************************