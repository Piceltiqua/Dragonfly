#ifndef EKF_H
#define EKF_H

#include <Eigen/LU>
#include <cmath>

#include "Utils.hpp"
using namespace Eigen;

#define N_STATE 6
#define N_GNSS 6
#define N_BARO 1
#define N_SIGMA (2 * N_STATE + 1)

struct EkfSnapshot {
    // From predict()
    float pos_N, pos_E, pos_D;
    float vel_N, vel_E, vel_D;
    float F_upper[21];  // Upper triangle of 6x6 matrix (21 elements)
    float Q_upper[21];  // Upper triangle of 6x6 matrix
    float P_upper[21];  // Upper triangle of 6x6 matrix

    // From updateGNSS()
    float y[6];         // Innovation vector
    float S_upper[21];  // Upper triangle of 6x6 innovation covariance
    float K[36];        // Full 6x6 Kalman gain (not symmetric)
    float I_upper[21];  // Upper triangle of 6x6 identity (sparse, mostly zeros)
};

class EKF {
public:
    EKF(PosVel& posvel,
        IMUAcceleration& imuAcc,
        GNSSData& gnssData) : posvel_(posvel),
                              imuAcc_(imuAcc),
                              gnssData_(gnssData),
                              last_snapshot_() {}

    void setup();
    void predict(float dt);
    void updateGNSS();
    const EkfSnapshot& getLastSnapshot() const { return last_snapshot_; }

private:
    MatrixXf computeQ(float dt);

    PosVel& posvel_;
    IMUAcceleration& imuAcc_;
    GNSSData& gnssData_;

    MatrixXf state;
    MatrixXf P;

    EkfSnapshot last_snapshot_;

    // Noise standard deviations
    float sigma_a = 0.35f;  // accelerometer noise: 0.35m/s²
    float s_a2 = sigma_a * sigma_a;
    float sigma_vel = 0.1f;
    float s_v2 = sigma_vel * sigma_vel;
    float dz_GNSS = 0.475f;
    float sigma_rot = 0.0872665f;  // rotation noise: 5° in radians
    float s_r2 = dz_GNSS * dz_GNSS * sigma_rot * sigma_rot;
    float sigma_gyro = 0.0541052f;  // gyro noise: 3.1°/s in rad/s
};

#endif
