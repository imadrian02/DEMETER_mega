#include "DifferentialDrive.h"
#include "variables.h"

DifferentialDrive::DifferentialDrive(int leftPwmPin, int leftDirectionPin, int rightPwmPin, int rightDirectionPin, int maxSpeed, int rampSteps)
  : leftMotor(leftPwmPin, leftDirectionPin), rightMotor(rightPwmPin, rightDirectionPin), maxSpeed(maxSpeed), rampSteps(rampSteps), rampCounter(0) {}

void DifferentialDrive::moveForward(float leftRatio, float rightRatio) {
  int leftSpeed = constrain(maxSpeed * leftRatio, 0, maxSpeed);
  int rightSpeed = constrain(maxSpeed * rightRatio, 0, maxSpeed);
  leftSpeed = (leftSpeed * rampCounter) / rampSteps;
  rightSpeed = (rightSpeed * rampCounter) / rampSteps;
  leftMotor.setSpeedAndDirection(leftSpeed, true);
  rightMotor.setSpeedAndDirection(rightSpeed, true);
  rampCounter = min(rampCounter + 1, rampSteps);
}

void DifferentialDrive::moveBackward(float leftRatio, float rightRatio) {
  int leftSpeed = constrain(maxSpeed * leftRatio, 0, maxSpeed);
  int rightSpeed = constrain(maxSpeed * rightRatio, 0, maxSpeed);
  leftSpeed = (leftSpeed * rampCounter) / rampSteps;
  rightSpeed = (rightSpeed * rampCounter) / rampSteps;
  leftMotor.setSpeedAndDirection(leftSpeed, false);
  rightMotor.setSpeedAndDirection(rightSpeed, false);
  rampCounter = min(rampCounter + 1, rampSteps);
}

void DifferentialDrive::turnLeft(float leftRatio, float rightRatio, bool forward) {
  int leftSpeed = constrain(maxSpeed * leftRatio, 0, maxSpeed);
  int rightSpeed = constrain(maxSpeed * rightRatio, 0, maxSpeed);
  leftSpeed = (leftSpeed * rampCounter) / rampSteps;
  rightSpeed = (rightSpeed * rampCounter) / rampSteps;
  if (forward) {
    leftMotor.setSpeedAndDirection(leftSpeed, true);
    rightMotor.setSpeedAndDirection(rightSpeed, true);
  } else {
    leftMotor.setSpeedAndDirection(leftSpeed, false);
    rightMotor.setSpeedAndDirection(rightSpeed, false);
  }
  rampCounter = min(rampCounter + 1, rampSteps);
}

void DifferentialDrive::turnRight(float leftRatio, float rightRatio, bool forward) {
  int leftSpeed = constrain(maxSpeed * leftRatio, 0, maxSpeed);
  int rightSpeed = constrain(maxSpeed * rightRatio, 0, maxSpeed);
  leftSpeed = (leftSpeed * rampCounter) / rampSteps;
  rightSpeed = (rightSpeed * rampCounter) / rampSteps;
  if (forward) {
    leftMotor.setSpeedAndDirection(leftSpeed, true);
    rightMotor.setSpeedAndDirection(rightSpeed, true);
  } else {
    leftMotor.setSpeedAndDirection(leftSpeed, false);
    rightMotor.setSpeedAndDirection(rightSpeed, false);
  }
  rampCounter = min(rampCounter + 1, rampSteps);
}

void DifferentialDrive::stop() {
  leftMotor.setSpeedAndDirection(0, true);
  rightMotor.setSpeedAndDirection(0, true);
  rampCounter = 0;
}
