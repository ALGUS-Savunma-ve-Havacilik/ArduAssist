#include "Arduino.h"
#include "ServoMod.h"

servoMinMax::servoMinMax(void)
{
  _iMaxPos = 90;
  _iMaxNeg = 90;
}

int servoMinMax::getMaxNeg()
{
  return _iMaxNeg;
}

int servoMinMax::getMaxPos()
{
  return _iMaxPos;
}

void servoMinMax::setMaxNeg(int iMax)
{
  _iMaxNeg = iMax;
}

void servoMinMax::setMaxPos(int iMax)
{
  _iMaxPos = iMax;
}

void servoMinMax::setMaxTravel(int iMax)
{
  _iMaxTravel = iMax;
}

int servoMinMax::getMaxTravel()
{
  return _iMaxTravel;
}

int servoMinMax::getProportional(int iInput)
{
  int iAbsTravel = _iMaxPos - _iMaxNeg;
  int iAbsInput = iInput - _iMaxNeg;
  double dPercent = (double)iAbsInput / (double)iAbsTravel;
  int iCalculated = (int)((double)_iMaxTravel * dPercent);
  return iCalculated;
}

ServoMod::ServoMod(void)
{
  _iOld = 90;
}

int ServoMod::noChange(int iInput) // Take in the value for a channel and pass it out unmodified
{
  return iInput;
}

int ServoMod::ch6Scaled(int iInput, int iChannel, int iCh6) //  Take in a value for a channel and scale it against channel 6.  100% = full data, 0% = no movement.
{
  int iNewValue = 0;
  int iCurValue = this->servos[iChannel].getProportional(iInput);
  int iServo6value = this->servos[scale].getProportional(iCh6);
  double dModPercent = (double)iServo6value / 180.0;
  iNewValue = (int)((double)iCurValue * dModPercent);
  return iNewValue;
}

int ServoMod::iLeftAileronSimple(int iInput)
{
  int iScaled = this->servos[roll].getProportional(iInput);
  return iScaled;
}

int ServoMod::iRightAileronSimple(int iInput)
{
  int iScaled = this->servos[roll].getProportional(iInput);
  return 180-iScaled;
}

void ServoMod::AileronComplex(int iInput, int *ileft, int *iRight, int iFlapTrim)
{
  int iScaled = this->servos[roll].getProportional(iInput);
  ileft = &iScaled;
  int right = 180 - iScaled;
  iRight = &right;
}