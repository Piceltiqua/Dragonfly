#ifndef LQR_attitude_HPP
#define LQR_attitude_HPP

#include "Utils.hpp"
#include <Eigen/LU>

class AttitudeController {
public:
    AttitudeController(Attitude& current_attitude,
        PosCtrlOutput& control_input,
        ActuatorCommands& attitute_control_output):
        current_attitude_(current_attitude),
        control_input_(control_input),
        attitute_control_output_(attitute_control_output){
            attitude_angle_.pitch = 0.0f;
            attitude_angle_.yaw = 0.0f;
            attitude_angle_.roll = 0.0f;
        }

    void init(Eigen::Matrix<float, 2, 4> K_init);
    void control();
    // void calibrate();
    void quat_to_Euler(Attitude& attitude_quat, AttitudeAngle& attitude_angle);

private:
    AttitudeAngle attitude_angle_;
    Attitude& current_attitude_;
    PosCtrlOutput& control_input_;
    ActuatorCommands& attitute_control_output_;

    // Un peu chelou cet déclaration
    Eigen::Matrix<float, 2, 4> K = 
    (Eigen::Matrix<float, 2, 4>() << 0.5345f, -0.0000f, 0.2484f, -0.0000f,
    -0.0000f, 0.5345f, -0.0000f, 0.2484f).finished(); 

    float offset_roll_ = 0.0f;
    float offset_ptich_ = 0.0f;
};

#endif