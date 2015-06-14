#ifndef SERVOMOD_H
#define SERVOMOD_H

#include "Arduino.h"

// As my radio does not send 0-180, rather 45-135 (sweep of 90 rather than 180, which the servo can do, I need to write a scale function.
// As I don't know what the absolute numbers are, we'll have to guess.  During startup, for all inputs move controls to extremes
// This will calibrate the min/max endpoints and we can do proportional calculations.  

class servoMinMax 
{
  public:
  servoMinMax(void);
  int getMaxPos();
  int getMaxNeg();
  void setMaxPos(int);
  void setMaxNeg(int);
  
  private:
  int _iMaxPos;
  int _iMaxNeg;
  
};

class ServoMod
{
  public:
  ServoMod(void);
  int noChange(int iInput);
  int ch6Scaled(int iInput, int iCh6);
  int iLeftAileronSimple(int iInput);
  int iRightAileronSimple(int iInput);
  void AileronComplex(int iInput, int *ileft, int *iRight, int iFlapTrim);
  servoMinMax servos[6];
  
  private:
  int _iOld;
};


#endif