#ifndef SERVOMOD_H
#define SERVOMOD_H

#include "Arduino.h"

class ServoMod
{
  public:
  ServoMod(void);
  int noChange(int iInput);
  int ch6Scaled(int iInput, int iCh6);
  int iLeftAileronSimple(int iInput);
  int iRightAileronSimple(int iInput);
  void AileronComplex(int iInput, int *ileft, int *iRight, int iFlapTrim);
  
  private:
  int _iOld;
  };


#endif