#ifndef ROLLCONTROL_H
#define ROLLCONTROL_H

#include "Utils.hpp"

class RollControl {
public:
    RollControl(ActuatorCommands& actuatorCmds) : actuatorCmds_(actuatorCmds) {}
    int computeRollTimingOffsets(float roll_rate_rps) {
        /*
        Adjust motor timings based on roll rate to stabilize the vehicle.
        roll_rate_rps: Current roll rate in radians per second.
        */

        return static_cast<int>(Kp_roll * roll_rate_rps + MOTOR_OFFSET);  // Offset in microseconds
    }

    static constexpr float MOTOR_OFFSET = 8.0f; 

private:
    ActuatorCommands& actuatorCmds_;

    static constexpr float Kp_roll = 50.0f;  // Proportional gain for roll control
};

#endif  // ROLLCONTROL_H