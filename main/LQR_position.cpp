#include "LQR_position.hpp"


LQR_position::LQR_position(){

}

void LQR_position::init(Eigen::Matrix<float, 2, 4> K_init){
    K = K_init;
}

void LQR_position::compute(const Attitude_angle& attitude, const rotationspeed& rotSpeed, const AttitudeSetpoint& attSetpoint, ControlOutput_attitude& ctrlOutput){
    /*
    The Function is the LQR controller for attitude control, it computes the control output based on the current attitude,
    */
    
    float pitch = attitude.pitch;
    float yaw = attitude.yaw;
    float rotSpeed_q = rotSpeed.q;
    float rotSpeed_r = rotSpeed.r;

    Eigen::Matrix<float, 4, 1> x;
    x << pitch, yaw, rotSpeed_q, rotSpeed_r;

    float pitch_sp = attSetpoint.pitch;
    float yaw_sp = attSetpoint.yaw;
    
    Eigen::Matrix<float, 4, 1> x_sp;
    // the desired angular rates are set to zero for attitude control because we want to stabilize the attitude
    x_sp <<  pitch_sp, yaw_sp, 0.0f, 0.0f;

    Eigen::Matrix<float, 4, 1> e;
    e = x - x_sp;

    Eigen::Matrix<float, 2, 1> u = -K * e;
    ctrlOutput.pitchOutput = u(0) * RAD_TO_DEG;
    ctrlOutput.yawOutput   = u(1) * RAD_TO_DEG;
};
