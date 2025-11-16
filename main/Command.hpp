#ifndef COMMAND_H
#define COMMAND_H

#include "Utils.hpp"
#include <Servo.h>

class Command {
public:
  Command() {}
  void setup();
  void commandGimbal(int angleX, int angleY);
  void commandMotors(int throttleMotor1, int throttleMotor2);
  void commandLegs(bool legsUp);
  void setLedColor(bool red, bool green, bool blue);

private:
  Servo leg1;
  Servo leg2;
  Servo leg3;

  static constexpr int RED_PIN = 35;
  static constexpr int GREEN_PIN = 34;
  static constexpr int BLUE_PIN = 33;

};

#endif