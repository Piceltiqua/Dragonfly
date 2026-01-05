#include "LQR_attitude.hpp"

void AttitudeController::init(Eigen::Matrix<float, 2, 4> K_init){
    K = K_init;
}

void AttitudeController::control(){
    quat_to_Euler(current_attitude_, attitude_angle_);
    // attitude_angle_.roll  -= offset_roll_;
    // attitude_angle_.pitch -= offset_pitch_;

    Eigen::Matrix<float, 4, 1> x;
    x << attitude_angle_.pitch, attitude_angle_.yaw, current_attitude_.wx, current_attitude_.wy;
    
    Eigen::Matrix<float, 4, 1> x_sp;
    // the target angular rates are set to zero
    x_sp <<  control_input_.attitudeSetpoint.pitch, control_input_.attitudeSetpoint.yaw, 0.0f, 0.0f;

    Serial.print("Pitch error: ");
    Serial.println(x(0) - x_sp(0));
    Serial.print("Yaw error: ");
    Serial.println(x(1) - x_sp(1));
    Serial.print("Wx error: ");
    Serial.println(x(3) - x_sp(3));
    Serial.print("Wy error: ");
    Serial.println(x(2) - x_sp(2));

    Eigen::Matrix<float, 4, 1> e = x - x_sp;
    Eigen::Matrix<float, 2, 1> u = -K * e;
    
    if (control_input_.thrustCommand > 0.001f) { 
        attitute_control_output_.servoXAngle = RAD_TO_DEG * u(1) / (control_input_.thrustCommand * control_input_.momentArm);
        attitute_control_output_.servoYAngle = - RAD_TO_DEG * u(0) / (control_input_.thrustCommand  * control_input_.momentArm);
    } else {
        attitute_control_output_.servoXAngle = 0.0f;
        attitute_control_output_.servoYAngle = 0.0f;
    }
};

// void AttitudeController::calibrate(){
//     float sum_roll = 0.0f;
//     float sum_pitch = 0.0f;
//     const int num_samples = 100;

//     for (int i = 0; i < num_samples; ++i) {
//         AttitudeAngle attitude_angle;
//         imu.read();
//         quaternion_to_Euler(current_attitude_, attitude_angle);
//         sum_roll += attitude_angle.roll;
//         sum_pitch += attitude_angle.pitch;
//         delay(2);
//     }

//     offset_roll_ = sum_roll / num_samples
//     offset_pitch_ = sum_pitch / num_samples
// }

void AttitudeController::quat_to_Euler(Attitude& attitude_quat, AttitudeAngle& attitude_angle) {
    
    float qw = attitude_quat.qw;
    float qi = attitude_quat.qi;
    float qj = attitude_quat.qj;
    float qk = attitude_quat.qk;

    double n = std::sqrt(qw * qw + qi * qi + qj * qj + qk * qk);

    if (n < 1e-12) {
        attitude_angle.roll = 0.0f;
        attitude_angle.pitch = 0.0f;
        attitude_angle.yaw = 0.0f;
        return;
    }

    float w = qw / n;
    float x = qi / n;
    float y = qj / n;
    float z = qk / n;

    float R11 = 1.0f - 2.0f * (y * y + z * z);
    float R12 = 2.0f * (x * y - w * z);
    float R21 = 2.0f * (x * y + w * z);
    float R22 = 1.0f - 2.0f * (x * x + z * z);
    float R31 = 2.0f * (x * z - w * y);
    float R32 = 2.0f * (y * z + w * x);
    float R33 = 1.0f - 2.0f * (x * x + y * y);

    float sin_pitch = R32;
    if (sin_pitch > 1.0f) sin_pitch = 1.0f;
    if (sin_pitch < -1.0f) sin_pitch = -1.0f;
    float pitch_x = std::asin(sin_pitch);
    float cos_pitch = std::cos(pitch_x);

    const float EPS = 1e-8f;
    float roll_z, yaw_y;
    
    if (std::fabs(cos_pitch) > EPS) {
        roll_z = std::atan2(-R12, R22);
        yaw_y = std::atan2(-R31, R33);
    } else {
        roll_z = std::atan2(R21, R11);
        yaw_y = 0.0f;
    }
    if (yaw_y < 0.0f) {
        yaw_y += PI;
    } else if (yaw_y > 0.0f) {
        yaw_y -= PI;
    }

    attitude_angle.roll =  static_cast<float>(roll_z);
    attitude_angle.pitch = - static_cast<float>(pitch_x);
    attitude_angle.yaw = static_cast<float>(yaw_y);
}