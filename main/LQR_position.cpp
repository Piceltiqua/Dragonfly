#include "LQR_position.hpp"

void PositionController::init(){
    x_intergral_ = 0.0f;
    y_intergral_ = 0.0f;
    z_intergral_ = 0.0f;
}

void PositionController::control(float dt){
    // Increment integral error
    x_intergral_ += (position_setpoint_.posN - current_gnss_.posN)*dt;
    y_intergral_ += (position_setpoint_.posE - current_gnss_.posE)*dt;
    z_intergral_ += - (position_setpoint_.posD - current_gnss_.posD)*dt;

    // Anti-windup
    if (x_intergral_ > 0.5f) x_intergral_ = 0.5f;
    if (x_intergral_ < -0.5f) x_intergral_ = -0.5f;
    if (y_intergral_ > 0.5f) y_intergral_ = 0.5f;
    if (y_intergral_ < -0.5f) y_intergral_ = -0.5f;
    if (z_intergral_ > 0.5f) z_intergral_ = 0.5f;
    if (z_intergral_ < -0.5f) z_intergral_ = -0.5f;

    Serial.print("PosN error: ");
    Serial.print(position_setpoint_.posN - current_gnss_.posN);
    Serial.print("\tVelN error: ");
    Serial.print(position_setpoint_.velN - current_gnss_.velN);
    Serial.print("\tPosN integral: ");
    Serial.println(x_intergral_);
    Serial.print("PosE error: ");
    Serial.print(position_setpoint_.posE - current_gnss_.posE);
    Serial.print("\tVelE error: ");
    Serial.print(position_setpoint_.velE - current_gnss_.velE);
    Serial.print("\tPosE integral: ");
    Serial.println(y_intergral_);
    Serial.print("PosD error: ");
    Serial.print(position_setpoint_.posD - current_gnss_.posD);
    Serial.print("\tVelD error: ");
    Serial.print(position_setpoint_.velD - current_gnss_.velD);
    Serial.print("\tPosD integral: ");
    Serial.println(z_intergral_);

    Eigen::Matrix<float, 3, 1> x;
    x << current_gnss_.posN,
         current_gnss_.velN,
         - x_intergral_;

    Eigen::Matrix<float, 3, 1> x_sp;
    x_sp << position_setpoint_.posN,
            position_setpoint_.velN,
            0;

    Eigen::Matrix<float, 3, 1> e_x = x - x_sp;
    Eigen::Matrix<float, 1, 1> u_x = -K_x_pos * e_x;

    Eigen::Matrix<float, 3, 1> y;
    y << current_gnss_.posE,
         current_gnss_.velE,
         - y_intergral_;

    Eigen::Matrix<float, 3, 1> y_sp;
    y_sp << position_setpoint_.posE,
            position_setpoint_.velE,
            0;

    Eigen::Matrix<float, 3, 1> e_y = y - y_sp;
    Eigen::Matrix<float, 1, 1> u_y = -K_y_pos * e_y;

    Eigen::Matrix<float, 3, 1> z;
    z << - current_gnss_.posD,
         - current_gnss_.velD,
         - z_intergral_;

    Eigen::Matrix<float, 3, 1> z_sp;
    z_sp << - position_setpoint_.posD,
            - position_setpoint_.velD,
            0;

    Eigen::Matrix<float, 3, 1> e_z = z - z_sp;
    Eigen::Matrix<float, 1, 1> u_z = -K_z_pos * e_z;

    // Convert acceleration command to thrust command
    
    position_control_output_.thrustCommand = sqrt((u_z(0)+ m*g)*(u_z(0)+ m*g)+u_y(0)*u_y(0)+u_x(0)*u_x(0));

    if (position_control_output_.thrustCommand == 0.0f){
        position_control_output_.attitudeSetpoint.pitch = 0.0f;
        position_control_output_.attitudeSetpoint.yaw = 0.0f;
        return;
    }

    float x_command =   u_x(0)*m / position_control_output_.thrustCommand;
    float y_command = - u_y(0)*m / position_control_output_.thrustCommand;
    position_control_output_.attitudeSetpoint.pitch = min( max(x_command, -0.3f), 0.3f);
    position_control_output_.attitudeSetpoint.yaw = min( max(y_command, -0.3f), 0.3f);

    Serial.print("Thrust command: ");
    Serial.println(position_control_output_.thrustCommand);
    Serial.print("Pitch command (X): ");
    Serial.println(position_control_output_.attitudeSetpoint.pitch);
    Serial.print("Roll command (Y): ");
    Serial.println(position_control_output_.attitudeSetpoint.yaw);
};