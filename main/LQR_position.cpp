#include "LQR_position.hpp"

void PositionController::init(){
    N_intergral_ = 0.0f;
    E_intergral_ = 0.0f;
    D_intergral_ = 0.0f;
}

void PositionController::control(float dt){
    // Increment integral error
    N_intergral_ +=   (position_setpoint_.posN - current_gnss_.posN)*dt;
    E_intergral_ +=   (position_setpoint_.posE - current_gnss_.posE)*dt;
    D_intergral_ += - (position_setpoint_.posD - current_gnss_.posD)*dt;

    // Anti-windup
    if (N_intergral_ > 0.5f) N_intergral_ = 0.5f;
    if (N_intergral_ < -0.5f) N_intergral_ = -0.5f;
    if (E_intergral_ > 0.5f) E_intergral_ = 0.5f;
    if (E_intergral_ < -0.5f) E_intergral_ = -0.5f;
    if (D_intergral_ > 0.5f) D_intergral_ = 0.5f;
    if (D_intergral_ < -0.5f) D_intergral_ = -0.5f;

    Serial.print("PosN error: ");
    Serial.print(position_setpoint_.posN - current_gnss_.posN);
    Serial.print("\tVelN error: ");
    Serial.print(position_setpoint_.velN - current_gnss_.velN);
    Serial.print("\tPosN integral: ");
    Serial.println(N_intergral_);
    Serial.print("PosE error: ");
    Serial.print(position_setpoint_.posE - current_gnss_.posE);
    Serial.print("\tVelE error: ");
    Serial.print(position_setpoint_.velE - current_gnss_.velE);
    Serial.print("\tPosE integral: ");
    Serial.println(E_intergral_);
    Serial.print("PosD error: ");
    Serial.print(position_setpoint_.posD - current_gnss_.posD);
    Serial.print("\tVelD error: ");
    Serial.print(position_setpoint_.velD - current_gnss_.velD);
    Serial.print("\tPosD integral: ");
    Serial.println(D_intergral_);

    Eigen::Matrix<float, 6, 1> NE;
    NE << current_gnss_.posN,   current_gnss_.posE,
          current_gnss_.velN,   current_gnss_.velE,
          N_intergral_,         E_intergral_;

    Eigen::Matrix<float, 6, 1> NE_sp;
    NE_sp << position_setpoint_.posN, position_setpoint_.posE,
             position_setpoint_.velN, position_setpoint_.velE,
             0,                       0;

    Eigen::Matrix<float, 6, 1> e_NE = NE - NE_sp;
    Eigen::Matrix<float, 2, 1> u_NE = -K_NE_pos * e_NE;

    Eigen::Matrix<float, 3, 1> D;
    D << - current_gnss_.posD,
         - current_gnss_.velD,
         - D_intergral_;

    Eigen::Matrix<float, 3, 1> D_sp;
    D_sp << - position_setpoint_.posD,
            - position_setpoint_.velD,
            0;

    Eigen::Matrix<float, 3, 1> e_D = D - D_sp;
    Eigen::Matrix<float, 1, 1> u_D = -K_D_pos * e_D;

    // Convert acceleration command to thrust command
    float thrust_N = m * sqrt((u_D(0) + g)*(u_D(0) + g)+u_NE(0)*u_NE(0)+u_NE(1)*u_NE(1));
    position_control_output_.thrustCommand = 1000*thrust_N/9.81f;

    if (position_control_output_.thrustCommand == 0.0f){
        position_control_output_.attitudeSetpoint.pitch = 0.0f;
        position_control_output_.attitudeSetpoint.yaw = 0.0f;
        return;
    }

    float N_command = u_NE(0)*m / thrust_N;
    float E_command = u_NE(1)*m / thrust_N;

    float pitch_command = ( - N_command * cos(DEG_TO_RAD * attitude_angle_.yaw)
                            - E_command * sin(DEG_TO_RAD * attitude_angle_.yaw));

    float yaw_command   = ( - N_command * sin(DEG_TO_RAD * attitude_angle_.yaw)
                            + E_command * cos(DEG_TO_RAD * attitude_angle_.yaw));

    position_control_output_.attitudeSetpoint.pitch = min(max(pitch_command, -0.3f), 0.3f);
    position_control_output_.attitudeSetpoint.yaw = min(max(yaw_command, -0.3f), 0.3f);

    Serial.print("Thrust command: ");
    Serial.println(position_control_output_.thrustCommand);
    Serial.print("N command: ");
    Serial.println(N_command);
    Serial.print("E command: ");
    Serial.println(E_command);
    Serial.print("Roll: ");
    Serial.println(attitude_angle_.roll);
    Serial.print("Pitch command (X): ");
    Serial.println(pitch_command);
    Serial.print("Roll command (Y): ");
    Serial.println(yaw_command);
};