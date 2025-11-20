#ifndef UKF_H
#define UKF_H

#include <Eigen/LU>
#include <cmath>

#include "Utils.hpp"
using namespace Eigen;

#define N_STATE 6
#define N_GNSS 6
#define N_BARO 1
#define N_SIGMA (2 * N_STATE + 1)

class UKF {
public:
    UKF(PosVel& posvel,
        Attitude& attitude,
        IMUAcceleration& imuAcc,
        GNSSData& gnssData) : posvel_(posvel),
                              attitude_(attitude),
                              imuAcc_(imuAcc),
                              gnssData_(gnssData) {}

    void setup();
    void predict(float dt);
    void updateGNSS();

private:
    void fx(Eigen::Ref<Eigen::MatrixXf> x, float dt);
    void generateSigmaPoints();
    void computeQ(float dt);

    template <int N>
    Eigen::Matrix<float, N, 1> weightedMean(const Eigen::Matrix<float, N, N_SIGMA>& sigmaMat);

    template <int N, int M>
    Eigen::Matrix<float, N, M> weightedCovariance(
        const Eigen::Matrix<float, N, N_SIGMA>& X_sigma,
        const Eigen::Matrix<float, N, 1>& x_mean,
        const Eigen::Matrix<float, M, N_SIGMA>& Y_sigma,
        const Eigen::Matrix<float, M, 1>& y_mean);

    PosVel& posvel_;
    Attitude& attitude_;
    IMUAcceleration& imuAcc_;
    GNSSData& gnssData_;

    MatrixXf state;
    MatrixXf x;  // estimated state vector

    // Rocket/IMU parameters
    float dz_IMU = 0.207f;
    float dz_GNSS = 0.475f;
    float sigma_a = 0.02f;
    float sigma_vel = 0.1f;

    // Sigma points
    MatrixXf sigma;
    float W_CENTRAL = 0.5f;
    float W_OTHER = 0.5f / (N_SIGMA - 1);
    static constexpr float SCALE = 0.775f;

    MatrixXf pos;
    MatrixXf vel;
    MatrixXf acc_body;
    MatrixXf acc_world;

    // Prediction covariance matrices
    MatrixXf R;
    MatrixXf P;
    MatrixXf Q;

    // GNSS update
    MatrixXf pos_GNSS;
    MatrixXf vel_GNSS;
    MatrixXf z_GNSS;
    MatrixXf R_GNSS;
    MatrixXf S_GNSS;
    MatrixXf K_GNSS;

    // Barometer update
};

#endif
