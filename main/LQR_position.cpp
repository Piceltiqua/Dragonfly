#include "LQR_position.hpp"

void PositionController::init(){
    integral_error_pos_ = Eigen::Matrix<float, 4, 1>::Zero();
}

void PositionController::control(){
    Eigen::Matrix<float, 6, 1> x;
    x << current_gnss_.posN, current_gnss_.posE, current_gnss_.posD,
         current_gnss_.velN, current_gnss_.velE, current_gnss_.velD;
    
    Eigen::Matrix<float, 6, 1> x_sp;
    x_sp << position_setpoint_.posN,
            position_setpoint_.posE,
            position_setpoint_.posD,
            position_setpoint_.velN,
            position_setpoint_.velE,
            position_setpoint_.velD;

    Eigen::Matrix<float, 6, 1> e = x - x_sp;
    Eigen::Matrix<float, 3, 1> u = -K_pos * e;

    // A voir pour l'ordre des commandes
    position_control_output_.thrustCommand = u(0);
    position_control_output_.attitudeSetpoint.pitch = u(1);
    position_control_output_.attitudeSetpoint.yaw = u(2);
};