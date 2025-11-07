#pragma once
#include <TinyMatrix.h>
#include "utils.hpp"

#define N_STATE 6
#define N_GNSS 3
#define N_BARO 1
#define N_LIDAR 1
#define N_SIGMA (2 * N_STATE + 1)

class UKF {
public:
    // Rocket characteristics
    float dz = 0.0f; // IMU offset along body Z
    float sigma_a = 0.02f; // Accelerometer std (m/s²)

    // State vector and covariance
    Matrix<N_STATE,1> pos; 
    Matrix<N_STATE,N_STATE> P;
    Matrix<N_STATE,N_STATE> Q;

    Matrix<N_GNSS,N_GNSS> R_GNSS;
    Matrix<N_BARO,N_BARO> R_BARO;
    Matrix<N_LIDAR,N_LIDAR> R_LIDAR;

    // Sigma points
    Matrix<N_STATE,1> sigma[N_SIGMA];

    // Weights
    float W_CENTRAL = 0.5f;
    float W_OTHER = 0.5f / (N_SIGMA-1);

    // Scale for sigma point generation
    static constexpr float SCALE = 0.775f;

    UKF();

    void predict(float ax, float ay, float az,
                 float q0, float q1, float q2, float q3,
                 float wx, float wy, float wz,
                 float dt);

    void updateGNSS(const Matrix<3,1> &z);
    void updateBarometer(const Matrix<3,1> &z);
    void updateLidar(const Matrix<1,1> &z);

private:
    void fx(Matrix<N_STATE,1> &x_in_out,
            float ax, float ay, float az,
            float q0, float q1, float q2, float q3,
            float wx, float wy, float wz,
            float dt);

    void generateSigmaPoints();
    void computeQ(float dt);
    void weightedMean(Matrix<N_STATE,1> &x);
    void weightedCovariance(const Matrix<N,N_SIGMA> &X_sigma,
                            const Matrix<N,1> &x_mean,
                            const Matrix<M,N_SIGMA> &Y_sigma,
                            const Matrix<M,1> &y_mean);
};
