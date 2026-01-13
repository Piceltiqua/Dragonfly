#ifndef LQR_position_HPP
#define LQR_position_HPP

#include "Utils.hpp"
#include <Eigen/LU>

class PositionController {
public:
    PositionController(GNSSData& current_gnss,
        PosCtrlSetpoint& position_setpoint,
        PosCtrlOutput& position_control_output):
        current_gnss_(current_gnss),
        position_setpoint_(position_setpoint),
        position_control_output_(position_control_output){}

    void init();
    void control(float dt);

private:
    GNSSData& current_gnss_;
    PosCtrlSetpoint& position_setpoint_;
    PosCtrlOutput& position_control_output_;

    // Positon, speed, error_integral
    Eigen::Matrix<float, 1, 3 > K_x_pos = (Eigen::Matrix<float, 1, 3>() << 1.1531f, 1.5268f, -0.3536f).finished();
    Eigen::Matrix<float, 1, 3 > K_y_pos = (Eigen::Matrix<float, 1, 3>() << 1.1531f, 1.5268f, -0.3536f).finished(); 
    Eigen::Matrix<float, 1, 3 > K_z_pos = (Eigen::Matrix<float, 1, 3>() << 4.3001f, 3.0167f, -2.2361f).finished(); 

    float x_intergral_ = 0.0f;
    float y_intergral_ = 0.0f;
    float z_intergral_ = 0.0f;

    float m = 1.3f; // mass in kg
    float g = 9.81f; // gravity in m/s^2
};

#endif