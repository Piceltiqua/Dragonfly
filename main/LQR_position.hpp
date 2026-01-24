#ifndef LQR_position_HPP
#define LQR_position_HPP

#include "Utils.hpp"
#include <Eigen/LU>

class PositionController {
public:
    PositionController(GNSSData& current_gnss,
        AttitudeAngle& attitude_angle,
        PosCtrlSetpoint& position_setpoint,
        PosCtrlOutput& position_control_output):
        current_gnss_(current_gnss),
        attitude_angle_(attitude_angle),
        position_setpoint_(position_setpoint),
        position_control_output_(position_control_output){}

    void init();
    void control(float dt);

    float N_integral_ = 0.0f;
    float E_integral_ = 0.0f;
    float D_integral_ = 0.0f;

private:
    GNSSData& current_gnss_;
    AttitudeAngle& attitude_angle_;
    PosCtrlSetpoint& position_setpoint_;
    PosCtrlOutput& position_control_output_;

    // Positon, speed, error_integral
    Eigen::Matrix<float, 2, 4 > K_NE_pos =
    (Eigen::Matrix<float, 2, 4>() << 0.55f,    0, 1.85f,    0,
                                        0, 0.55f,    0, 1.85f).finished();

    Eigen::Matrix<float, 1, 3 > K_D_pos = 
    (Eigen::Matrix<float, 1, 3>() << 0.25f, 1.1f, 0.05f).finished(); 

    float m = 1.334f; // mass in kg
    float g = 9.81f; // gravity in m/s^2
};

#endif