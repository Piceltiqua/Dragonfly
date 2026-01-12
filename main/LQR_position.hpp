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
    void control();

private:
    GNSSData& current_gnss_;
    PosCtrlSetpoint& position_setpoint_;
    PosCtrlOutput& position_control_output_;

    // Adapter la taille de la matrice je suis pas sûre
    Eigen::Matrix<float, 3, 6> K_pos;
    // Add Matrix initialization here
    // Add integral matrix*
    Eigen::Matrix<float, 1, 3 > K_x_pos = (Eigen::Matrix<float, 1, 3>() << 1.1531f, 1.5268f, -0.3536f).finished();

    Eigen::Matrix<float, 1, 3 > K_y_pos = (Eigen::Matrix<float, 1, 3>() << 1.1531f, 1.5268f, -0.3536f).finished(); 
    
    Eigen::Matrix<float, 1, 3 > K_z_pos = (Eigen::Matrix<float, 1, 3>() << 4.3001f, 3.0167f, -2.2361f).finished(); 

    Eigen::Matrix<float, 4, 1> integral_error_pos_;
};

#endif