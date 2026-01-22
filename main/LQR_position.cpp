#include "LQR_position.hpp"

void PositionController::init(){
    N_integral_ = 0.0f;
    E_integral_ = 0.0f;
    D_integral_ = 0.0f;
}

void PositionController::control(float dt){
    // Increment integral error
    // N_integral_ +=   (position_setpoint_.posN - current_gnss_.posN)*dt;
    // E_integral_ +=   (position_setpoint_.posE - current_gnss_.posE)*dt;
    D_integral_ += - (position_setpoint_.posD - current_gnss_.posD)*dt;

    // Anti-windup
    // if (N_integral_ > 0.5f) N_integral_ = 0.5f;
    // if (N_integral_ < -0.5f) N_integral_ = -0.5f;
    // if (E_integral_ > 0.5f) E_integral_ = 0.5f;
    // if (E_integral_ < -0.5f) E_integral_ = -0.5f;
    if (D_integral_ > 2.0f) D_integral_ = 2.0f;
    if (D_integral_ < -2.0f) D_integral_ = -2.0f;

    Serial.print("PosN error: ");
    Serial.print(position_setpoint_.posN - current_gnss_.posN);
    Serial.print("\tVelN error: ");
    Serial.println(position_setpoint_.velN - current_gnss_.velN);
    Serial.print("\tPosN integral: ");
    Serial.println(N_integral_);
    Serial.print("PosE error: ");
    Serial.print(position_setpoint_.posE - current_gnss_.posE);
    Serial.print("\tVelE error: ");
    Serial.println(position_setpoint_.velE - current_gnss_.velE);
    Serial.print("\tPosE integral: ");
    Serial.println(E_integral_);
    Serial.print("PosD error: ");
    Serial.print(position_setpoint_.posD - current_gnss_.posD);
    Serial.print("\tVelD error: ");
    Serial.print(position_setpoint_.velD - current_gnss_.velD);
    Serial.print("\tPosD integral: ");
    Serial.println(D_integral_);

    Eigen::Matrix<float, 4, 1> NE;
    NE << current_gnss_.posN,   current_gnss_.posE,
          current_gnss_.velN,   current_gnss_.velE;
        //   N_integral_,         E_integral_;

    Eigen::Matrix<float, 4, 1> NE_sp;
    NE_sp << position_setpoint_.posN, position_setpoint_.posE,
             position_setpoint_.velN, position_setpoint_.velE;
            //  0,                       0;

    Eigen::Matrix<float, 4, 1> e_NE = NE - NE_sp;
    Eigen::Matrix<float, 2, 1> u_NE = -K_NE_pos * e_NE;

    Eigen::Matrix<float, 3, 1> D;
    D << - current_gnss_.posD,
         - current_gnss_.velD,
         - D_integral_;

    Eigen::Matrix<float, 3, 1> D_sp;
    D_sp << - position_setpoint_.posD,
            - position_setpoint_.velD,
            0;

    Eigen::Matrix<float, 3, 1> e_D = D - D_sp;
    Eigen::Matrix<float, 1, 1> u_D = -K_D_pos * e_D;

    float accN = u_NE(0); // + position_setpoint_.accffN;
    float accE = u_NE(1); // + position_setpoint_.accffE;
    float accD = u_D(0) - position_setpoint_.accffD; // Add feedforward acceleration
    accD = min(max(accD, -g), 20.60f); // prevent negative total thrust

    // Convert acceleration command to thrust command
    float thrust_N = m * sqrt((accD + g)*(accD + g)+ accN*accN + accE*accE);
    position_control_output_.thrustCommand = 1000*thrust_N/9.81f;

    if (position_control_output_.thrustCommand == 0.0f){
        position_control_output_.attitudeSetpoint.pitch = 0.0f;
        position_control_output_.attitudeSetpoint.yaw = 0.0f;
        return;
    }

    float N_command = accN*m / thrust_N;
    float E_command = accE*m / thrust_N;

    float yaw_command = ( - N_command * cos(attitude_angle_.roll)
                          - E_command * sin(attitude_angle_.roll));

    float pitch_command = ( N_command * sin(attitude_angle_.roll)
                          - E_command * cos(attitude_angle_.roll));

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
    Serial.print(pitch_command);
    Serial.print("\t Pitch: ");
    Serial.println(attitude_angle_.pitch);
    Serial.print("Yaw command (Y): ");
    Serial.print(yaw_command);
    Serial.print("\t Yaw: ");
    Serial.println(attitude_angle_.yaw);
};