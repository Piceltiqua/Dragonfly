#ifndef COMMAND_H
#define COMMAND_H

#include "Utils.hpp"

class Command {
public:
    Command() {}
    void commandGimbal(int angleX, int angleY);
    void commandMotors(int throttleMotor1, int throttleMotor2);
    void commandLegs(bool legsUp);
}

#endif