#include "LQR_attitude.hpp"


LQR_attitude::LQR_attitude(){

}

void LQR_attitude::init(Eigen::Matrix<float, 2, 4> K_init){
    K = K_init;
}

void LQR_attitude::compute(const Attitude_angle& attitude, const rotationspeed& rotSpeed,float thrust, float moment_arm, const AttitudeSetpoint& attSetpoint, ControlOutput_attitude& ctrlOutput){
    /*
    The Function is the LQR controller for attitude control, it computes the control output based on the current attitude,
    */
    
    float pitch = attitude.pitch;
    float yaw = attitude.yaw;
    float rotSpeed_q = rotSpeed.q;
    float rotSpeed_r = rotSpeed.r;
    float torque_x;
    float torque_y;
    Eigen::Matrix<float, 4, 1> x;
    x << pitch, yaw, rotSpeed_q, rotSpeed_r;

    float pitch_sp = attSetpoint.pitch;
    float yaw_sp = attSetpoint.yaw;
    
    Eigen::Matrix<float, 4, 1> x_sp;
    // the desired angular rates are set to zero for attitude control because we want to stabilize the attitude
    x_sp <<  pitch_sp, yaw_sp, 0.0f, 0.0f;

    Eigen::Matrix<float, 4, 1> e;
    e = x - x_sp;

    Eigen::Matrix<float, 2, 1> u = -K * e; // that give the control inputs (torques)
    //now we need to convert the torques to control outputs (servo angles)
    torque_x = u(0);
    torque_y = u(1);
    // Convert torques to gimbal angles

    
    if (thrust > 0.001f) { 
        ctrlOutput.pitchOutput = -torque_x / (thrust * moment_arm);
        ctrlOutput.yawOutput = torque_y / (thrust * moment_arm);
    } else {
        ctrlOutput.pitchOutput = 0.0f;
        ctrlOutput.yawOutput = 0.0f;
    }
};
