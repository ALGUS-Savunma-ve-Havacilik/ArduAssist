/*
 * ArduAssist.h
 *
 * Created: 17/06/2015 11:10:21
 *  Author: Richard
 */ 


#ifndef ARDUASSIST_H_
#define ARDUASSIST_H_


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


#endif /* ARDUASSIST_H_ */