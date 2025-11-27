#include "EKF.hpp"

void EKF::setup() {
    state = MatrixXf::Zero(N_STATE, 1);

    float pos_var = 10.0f;   // (m^2)
    float vel_var = 1.0f;    // (m^2/s^2)

    P = Matrix<float, N_STATE, N_STATE>::Zero();
    P.block<3,3>(0,0) = Matrix3f::Identity() * pos_var;
    P.block<3,3>(3,3) = Matrix3f::Identity() * vel_var;
}

void EKF::predict(float dt) {
    // 1) Extract and update state using IMU acceleration
    state << posvel_.posN, posvel_.posE, posvel_.posD,
             posvel_.velN, posvel_.velE, posvel_.velD;
    
    Vector3f pos = state.block<3,1>(0,0);
    Vector3f vel = state.block<3,1>(3,0);

    Vector3f acc( imuAcc_.ax_NED,
                  imuAcc_.ay_NED,
                  imuAcc_.az_NED );

    // Kinematics:
    // vel_new = vel_old + a * dt
    // pos_new = pos_old + vel_old * dt + 0.5 * a * dt^2
    pos = pos + vel * dt + 0.5f * acc * dt * dt;
    vel = vel + acc * dt;

    state.block<3,1>(0,0) = pos;
    state.block<3,1>(3,0) = vel;
    
    // 2) Build state transition matrix F
    Matrix<float, N_STATE, N_STATE> F = Matrix<float, N_STATE, N_STATE>::Identity();
    F.block<3,3>(0,3) = Matrix3f::Identity() * dt;


    // 3) Build process noise matrix Q
    MatrixXf Q = computeQ(dt);


    // 4) Propagate covariance
    //          P = F P Fᵀ + Q
    P = F * P * F.transpose() + Q;


    // 5) Update external output struct
    posvel_.posN = state(0, 0);
    posvel_.posE = state(1, 0);
    posvel_.posD = state(2, 0);
    posvel_.velN = state(3, 0);
    posvel_.velE = state(4, 0);
    posvel_.velD = state(5, 0);
}

void EKF::updateGNSS() {
    // 1) Build measurement vector z
    Matrix<float, N_GNSS, 1> z;
    z << gnssData_.posN,
         gnssData_.posE,
         gnssData_.posD,
         gnssData_.velN,
         gnssData_.velE,
         gnssData_.velD;

    // 2) Measurement covariance R
    Matrix<float, N_GNSS, N_GNSS> R = Matrix<float, N_GNSS, N_GNSS>::Zero();
    R(0,0) = gnssData_.horAcc*gnssData_.horAcc + s_r2; 
    R(1,1) = gnssData_.horAcc*gnssData_.horAcc + s_r2;
    R(2,2) = gnssData_.vertAcc*gnssData_.vertAcc;
    R(3,3) = s_v2;
    R(4,4) = s_v2;
    R(5,5) = s_v2;

    // 3) Innovation y = z − x
    Matrix<float, N_GNSS, 1> y = z - state;

    // 4) Innovation covariance S
    Matrix<float, N_GNSS, N_GNSS> S = P + R;

    // 5) Kalman gain K
    Matrix<float, N_STATE, N_GNSS> K = P * S.inverse();

    // 6) State update
    state = state + K * y;

    // 7) Covariance update
    Matrix<float, N_STATE, N_STATE> I = Matrix<float, N_STATE, N_STATE>::Identity();
    P = (I - K) * P;

    // 8) Update external output struct
    posvel_.posN = state(0, 0);
    posvel_.posE = state(1, 0);
    posvel_.posD = state(2, 0);
    posvel_.velN = state(3, 0);
    posvel_.velE = state(4, 0);
    posvel_.velD = state(5, 0);
}

MatrixXf EKF::computeQ(float dt) {
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    // Discrete-time constant-acceleration model per axis:
    float q_pp = dt4 * 0.25f * s_a2;   // pos-pos
    float q_pv = dt3 * 0.5f  * s_a2;   // pos-vel and vel-pos
    float q_vv = dt2          * s_a2;  // vel-vel

    MatrixXf Q = MatrixXf::Zero(N_STATE, N_STATE);
    Q.block<3,3>(0,0) = Matrix3f::Identity() * q_pp;
    Q.block<3,3>(0,3) = Matrix3f::Identity() * q_pv;
    Q.block<3,3>(3,0) = Matrix3f::Identity() * q_pv;
    Q.block<3,3>(3,3) = Matrix3f::Identity() * q_vv;

    return Q;
}