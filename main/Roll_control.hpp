#ifndef ROLLCONTROL_H
#define ROLLCONTROL_H

#include "Utils.hpp"

class RollControl {
public:
    RollControl(ActuatorCommands& actuatorCmds) : actuatorCmds_(actuatorCmds) {}
    int computeRollTimingOffsets(float roll_rate_rps, uint8_t legsPosition) {
        /*
        Adjust motor timings based on roll rate to stabilize the vehicle.
        roll_rate_rps: Current roll rate in radians per second.
        */
        if (legsPosition == LEGS_DEPLOYED) {
            return static_cast<int>(Kp_deployed_roll * roll_rate_rps + MOTOR_DEPLOYED_OFFSET);
        } else {
            return static_cast<int>(Kp_retracted_roll * roll_rate_rps + MOTOR_RETRACTED_OFFSET);
        }
    }

    static constexpr float MOTOR_DEPLOYED_OFFSET  = 8.0f;
    static constexpr float MOTOR_RETRACTED_OFFSET = 3.0f;

private:
    ActuatorCommands& actuatorCmds_;

    static constexpr float Kp_deployed_roll  = 50.0f;  // Proportional gain for roll control
    static constexpr float Kp_retracted_roll = 12.0f;  // Proportional gain for roll control
};

#endif  // ROLLCONTROL_H