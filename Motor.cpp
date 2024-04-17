#include "Motor.h"
#include "variables.h"

Motor::Motor(int pwmPin, int directionPin)
  : pwmPin(pwmPin), directionPin(directionPin) {
  pinMode(pwmPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
}

void Motor::setSpeedAndDirection(int speed, bool forward) {
  digitalWrite(directionPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, speed);
}
