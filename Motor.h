#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
  int pwmPin;
  int directionPin;

public:
  Motor(int pwmPin, int directionPin);
  void setSpeedAndDirection(int speed, bool forward);
};

#endif
