#ifndef DIFFERENTIALDRIVE_H
#define DIFFERENTIALDRIVE_H

#include <Arduino.h>
#include "Motor.h"

class DifferentialDrive {
private:
  Motor leftMotor;
  Motor rightMotor;
  int maxSpeed;
  int rampSteps;
  int rampCounter;

public:
  DifferentialDrive(int leftPwmPin, int leftDirectionPin, int rightPwmPin, int rightDirectionPin, int maxSpeed, int rampSteps);
  void moveForward(float leftRatio, float rightRatio);
  void moveBackward(float leftRatio, float rightRatio);
  void turnLeft(float leftRatio, float rightRatio, bool forward);
  void turnRight(float leftRatio, float rightRatio, bool forward);
  void stop();
};

#endif
