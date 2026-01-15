#ifndef LQR_attitude_HPP
#define LQR_attitude_HPP

#include "Utils.hpp"
#include <Eigen/LU>

class AttitudeController {
public:
    AttitudeController(Attitude& current_attitude,
        AttitudeAngle& attitude_angle,
        PosCtrlOutput& attitude_setpoint,
        ActuatorCommands& attitute_control_output):
        current_attitude_(current_attitude),
        attitude_angle_(attitude_angle),
        attitude_setpoint_(attitude_setpoint),
        attitute_control_output_(attitute_control_output){
            attitude_angle_.pitch = 0.0f;
            attitude_angle_.yaw = 0.0f;
            attitude_angle_.roll = 0.0f;
        }

    void control();
    void quat_to_Euler(Attitude& attitude_quat, AttitudeAngle& attitude_angle);

private:
    Attitude& current_attitude_;
    AttitudeAngle& attitude_angle_;
    PosCtrlOutput& attitude_setpoint_;
    ActuatorCommands& attitute_control_output_;

    Eigen::Matrix<float, 2, 4> K_att = 
    (Eigen::Matrix<float, 2, 4>() << 0.5345f, -0.0000f,  0.2484f, -0.0000f,
                                    -0.0000f,  0.5345f, -0.0000f,  0.2484f).finished();
};

#endif