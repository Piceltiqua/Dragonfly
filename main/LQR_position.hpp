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

private:
    GNSSData& current_gnss_;
    AttitudeAngle& attitude_angle_;
    PosCtrlSetpoint& position_setpoint_;
    PosCtrlOutput& position_control_output_;

    // Positon, speed, error_integral
    Eigen::Matrix<float, 2, 6 > K_NE_pos =
    (Eigen::Matrix<float, 2, 6>() << 1.1531f, 0, 1.5268f, 0, -0.3536f, 0,
                                     0, 1.1531f, 0, 1.5268f, 0, -0.3536f).finished();
    Eigen::Matrix<float, 1, 3 > K_D_pos = 
    (Eigen::Matrix<float, 1, 3>() << 4.3001f, 3.0167f, -2.2361f).finished(); 

    float N_intergral_ = 0.0f;
    float E_intergral_ = 0.0f;
    float D_intergral_ = 0.0f;

    float m = 1.3f; // mass in kg
    float g = 9.81f; // gravity in m/s^2
};

#endif