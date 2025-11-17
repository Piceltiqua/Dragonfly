#include "UKF.hpp"

void UKF::setup()
{
    state       = MatrixXf::Zero(N_STATE, 1);
    x           = MatrixXf::Zero(N_STATE, 1);
    sigma       = MatrixXf::Zero(N_STATE, N_SIGMA);

    pos         = MatrixXf::Zero(3,1);
    vel         = MatrixXf::Zero(3,1);
    acc_body    = MatrixXf::Zero(3,1);
    acc_world   = MatrixXf::Zero(3,1);

    R           = MatrixXf::Zero(3,3);
    P           = MatrixXf::Identity(N_STATE, N_STATE);
    Q           = MatrixXf::Zero(N_STATE, N_STATE);

    z_GNSS      = MatrixXf::Zero(N_GNSS,1);
    pos_GNSS    = MatrixXf::Zero(3,1);
    vel_GNSS    = MatrixXf::Zero(3,1);
    R_GNSS      = MatrixXf::Zero(N_GNSS,N_GNSS);
    R_GNSS(3,3) = sigma_vel * sigma_vel;
    R_GNSS(4,4) = sigma_vel * sigma_vel;
    R_GNSS(5,5) = sigma_vel * sigma_vel;
    S_GNSS      = MatrixXf::Zero(N_GNSS,N_GNSS);
    K_GNSS      = MatrixXf::Zero(N_STATE,N_GNSS);

    z_BARO      = 0.0f;
    S_BARO      = 0.0f;
    K_BARO      = MatrixXf::Zero(N_STATE,1);
}

void UKF::predict(float dt)
{
    state << posvel_.posN, posvel_.posE, posvel_.posD,
             posvel_.velN, posvel_.velE, posvel_.velD;

    generateSigmaPoints();

    for (int i = 0; i < N_SIGMA; i++) {
        fx(sigma.col(i), dt);
    }

    x = weightedMean<N_STATE>(sigma);
    P = weightedCovariance<N_STATE, N_STATE>(sigma, x, sigma, x);

    computeQ(dt);
    P += Q;

    // update the posvel struct
    posvel_.posN = x(0,0);
    posvel_.posE = x(1,0);
    posvel_.posD = x(2,0);
    posvel_.velN = x(3,0);
    posvel_.velE = x(4,0);
    posvel_.velD = x(5,0);
}

void UKF::updateGNSS()
{
    float cx = 2.f * (attitude_.qi*attitude_.qk + attitude_.qw*attitude_.qj) * dz_GNSS;
    float cy = 2.f * (attitude_.qj*attitude_.qk - attitude_.qw*attitude_.qi) * dz_GNSS;
    float cz = (1.f - 2.f*(attitude_.qi*attitude_.qi + attitude_.qj*attitude_.qj)) * dz_GNSS;

    // Corrected position
    pos_GNSS <<  gnssData_.posN - cx,
               gnssData_.posE - cy,
               gnssData_.posD - cz;

    float dvx = attitude_.wy * cz - attitude_.wz * cy;
    float dvy = attitude_.wz * cx - attitude_.wx * cz;
    float dvz = attitude_.wx * cy - attitude_.wy * cx;

    // Corrected velocity
    vel_GNSS <<  gnssData_.velN - dvx,
               gnssData_.velE - dvy,
               gnssData_.velD - dvz;


    // Update measurement vector for UKF
    z_GNSS.block(0,0,3,1) = pos_GNSS;
    z_GNSS.block(3,0,3,1) = vel_GNSS;

    R_GNSS(0,0) = gnssData_.horAcc * gnssData_.horAcc;
    R_GNSS(1,1) = gnssData_.horAcc * gnssData_.horAcc;
    R_GNSS(2,2) = gnssData_.vertAcc * gnssData_.vertAcc;

    S_GNSS = P + R_GNSS;
    K_GNSS = P * S_GNSS.inverse();

    x = x + K_GNSS * (z_GNSS - x);
    P = P - K_GNSS * S_GNSS * K_GNSS.transpose();

    posvel_.posN = x(0,0);
    posvel_.posE = x(1,0);
    posvel_.posD = x(2,0);
    posvel_.velN = x(3,0);
    posvel_.velE = x(4,0);
    posvel_.velD = x(5,0);
}

void UKF::updateBarometer()
{
    z_BARO = barometerData_.altBaro;

    S_BARO = P(2,2) + R_BARO;
    K_BARO = P.col(2) / S_BARO;
    x += K_BARO * (z_BARO - x(2,0));
    P -= K_BARO * K_BARO.transpose() * S_BARO;

    posvel_.posD = x(2,0);
}

void UKF::fx(Eigen::Ref<Eigen::MatrixXf> x, float dt)
{
    pos = x.block(0,0,3,1);
    vel = x.block(3,0,3,1);

    acc_body(0,0) = imuAcc_.ax - (-dz_IMU * (attitude_.wy*attitude_.wy + attitude_.wz*attitude_.wz));
    acc_body(1,0) = imuAcc_.ay - ( dz_IMU * (attitude_.wx*attitude_.wy));
    acc_body(2,0) = imuAcc_.az;

    R(0,0) =  2*(attitude_.qi*attitude_.qj + attitude_.qw*attitude_.qk);
    R(0,1) =  1 - 2*(attitude_.qi*attitude_.qi + attitude_.qk*attitude_.qk);
    R(0,2) =  2*(attitude_.qj*attitude_.qk - attitude_.qw*attitude_.qi);

    R(1,0) =  1 - 2*(attitude_.qj*attitude_.qj + attitude_.qk*attitude_.qk);
    R(1,1) =  2*(attitude_.qi*attitude_.qj - attitude_.qw*attitude_.qk);
    R(1,2) =  2*(attitude_.qi*attitude_.qk + attitude_.qw*attitude_.qj);

    R(2,0) = -2*(attitude_.qi*attitude_.qk - attitude_.qw*attitude_.qj);
    R(2,1) = -2*(attitude_.qj*attitude_.qk + attitude_.qw*attitude_.qi);
    R(2,2) = -(1 - 2*(attitude_.qi*attitude_.qi + attitude_.qj*attitude_.qj));

    acc_world = R * acc_body;
    pos += vel * dt + acc_world * (0.5f * dt * dt);
    vel += acc_world * dt;

    x.block(0,0,3,1) = pos;
    x.block(3,0,3,1) = vel;
}

void UKF::generateSigmaPoints()
{
    sigma.col(0) = x;

    for (int i = 0; i < N_STATE; ++i)
    {
        float std_i = std::sqrt(std::max(P(i, i), 0.0f));

        Eigen::Matrix<float, N_STATE, 1> delta;
        delta.setZero();
        delta(i,0) = SCALE * std_i;

        sigma.col(1 + i)         = x + delta;
        sigma.col(1 + N_STATE + i) = x - delta;
    }
}

void UKF::computeQ(float dt)
{
    float dt2 = dt*dt;
    float dt3 = dt2*dt;
    float dt4 = dt2*dt2;
    float s2  = sigma_a*sigma_a;

    MatrixXf Q_axis(2,2);
    Q_axis << 0.25f*dt4*s2, 0.5f*dt3*s2,
              0.5f*dt3*s2, dt2*s2;

    Q.setZero();
    Q.block(0,0,2,2) = Q_axis;
    Q.block(2,2,2,2) = Q_axis;
    Q.block(4,4,2,2) = Q_axis;
}

template<int N>
Eigen::Matrix<float, N, 1> UKF::weightedMean(const Eigen::Matrix<float, N, N_SIGMA> &sigmaMat)
{
    Eigen::Matrix<float, N, 1> x_mean = W_CENTRAL * sigmaMat.col(0);
    for(int i = 1; i < N_SIGMA; i++)
        x_mean += W_OTHER * sigmaMat.col(i);
    return x_mean;
}

template<int N, int M>
Eigen::Matrix<float, N, M> UKF::weightedCovariance(
        const Eigen::Matrix<float, N, N_SIGMA> &X_sigma,
        const Eigen::Matrix<float, N, 1>       &x_mean,
        const Eigen::Matrix<float, M, N_SIGMA> &Y_sigma,
        const Eigen::Matrix<float, M, 1>       &y_mean)
{
    Eigen::Matrix<float, N, M> P_xy = Eigen::Matrix<float, N, M>::Zero();
    for(int i = 0; i < N_SIGMA; i++) {
        Eigen::Matrix<float, N, 1> dx = X_sigma.col(i) - x_mean;
        Eigen::Matrix<float, M, 1> dy = Y_sigma.col(i) - y_mean;
        float w = (i == 0) ? W_CENTRAL : W_OTHER;
        P_xy += w * (dx * dy.transpose());
    }
    return P_xy;
}
