#include "UKF.h"
#include <math.h>

UKF::UKF()
{
    state.zeros();
    P.eye();

    R_GNSS.zeros();

    R_Baro.zeros();
    R_BARO(0,0) = baro_var_z;
}

void UKF::fx(Matrix<N_STATE,1> &x,
             float ax, float ay, float az,
             float q0, float q1, float q2, float q3,
             float wx, float wy, float wz,
             float dt)
{
    // Split state into position and velocity
    Matrix<3,1> pos, vel, acc_body, acc_world;

    for(int i=0;i<3;i++){
        pos(i,0) = x(i,0);
        vel(i,0) = x(i+3,0);
    }

    // Acceleration with rotational correction
    acc_body(0,0) = ax - (-dz*(wy*wy + wz*wz));
    acc_body(1,0) = ay - ( dz*(wx*wy));
    acc_body(2,0) = az;

    // Rotation matrix from quaternion
    Matrix<3,3> R;
    R(0,0) = 1 - 2*(q2*q2 + q3*q3);  R(0,1) = 2*(q1*q2 - q0*q3);     R(0,2) = 2*(q1*q3 + q0*q2);
    R(1,0) = 2*(q1*q2 + q0*q3);      R(1,1) = 1 - 2*(q1*q1 + q3*q3); R(1,2) = 2*(q2*q3 - q0*q1);
    R(2,0) = 2*(q1*q3 - q0*q2);      R(2,1) = 2*(q2*q3 + q0*q1);     R(2,2) = 1 - 2*(q1*q1 + q2*q2);

    // Transform acceleration to world frame and remove gravity
    acc_world = R * acc_body;
    acc_world(2,0) -= 9.81f;

    // Integrate position and velocity
    pos += vel * dt + 0.5f * acc_world * dt * dt;
    vel += acc_world * dt;

    // Write back into state vector
    x.submatrix(0,0,3,1) = pos;
    x.submatrix(3,0,3,1) = vel;
}

void UKF::generateSigmaPoints()
{
    // Central point
    sigma[0] = state;

    // Standard deviations along each axis
    for(int i=0; i<N_STATE; i++){
        float std_i = sqrtf(fmaxf(P(i,i), 0.0f));
        for(int j=0; j<N_STATE; j++){
            float delta = (j==i) ? SCALE*std_i : 0.0f;
            sigma[1+i](j,0) = state(j,0) + delta;
            sigma[1+N_STATE+i](j,0) = state(j,0) - delta;
        }
    }
}

void UKF::computeQ(float dt)
{
    // Precompute powers of dt
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt2 * dt2;

    // 2x2 block for one axis
    Matrix<2,2> Q_axis;
    Q_axis(0,0) = 0.25f * dt4 * sigma_a*sigma_a;
    Q_axis(0,1) = 0.5f  * dt3 * sigma_a*sigma_a;
    Q_axis(1,0) = 0.5f  * dt3 * sigma_a*sigma_a;
    Q_axis(1,1) = dt2   * sigma_a*sigma_a;

    // Assign blocks to x, y, z axes
    Q.submatrix(0,0,2,2) = Q_axis;  // x-axis
    Q.submatrix(2,2,2,2) = Q_axis;  // y-axis
    Q.submatrix(4,4,2,2) = Q_axis;  // z-axis
}

template<int N>
Matrix<N,1> UKF::weightedMean(const Matrix<N,N_SIGMA> &sigmaMat) {
    Matrix<N,1> x_mean = W_CENTRAL * sigmaMat.col(0);
    for(int i=1;i<N_SIGMA;i++)
        x_mean += W_OTHER * sigmaMat.col(i);
    return x_mean;
}

template<int N, int M>
Matrix<N,M> UKF::weightedCovariance(const Matrix<N,N_SIGMA> &X_sigma,
                                    const Matrix<N,1> &x_mean,
                                    const Matrix<M,N_SIGMA> &Y_sigma,
                                    const Matrix<M,1> &y_mean) {
    Matrix<N,M> P_xy; P_xy.zeros();
    for(int i=0;i<N_SIGMA;i++){
        Matrix<N,1> dx = X_sigma.col(i) - x_mean;
        Matrix<M,1> dy = Y_sigma.col(i) - y_mean;
        float w = (i==0)? W_CENTRAL : W_OTHER;
        P_xy += w * (dx * dy.transpose());
    }
    return P_xy;
}

void UKF::predict(float ax, float ay, float az,
                  float q0, float q1, float q2, float q3,
                  float wx, float wy, float wz,
                  float dt)
{
    generateSigmaPoints();

    for(int i=0; i<N_SIGMA; i++){
        fx(sigma[i], ax, ay, az, q0, q1, q2, q3, wx, wy, wz, dt);
    }

    state = weightedMean<N_STATE>(X_sigma_pred);
    P = weightedCovariance<N_STATE, N_STATE>(X_sigma_pred, state, X_sigma_pred, state);

    computeQ(dt);
    P += Q;
}

void UKF::updateGNSS(const Matrix<3,1> &z_meas, float horizontal_accuracy, float vertical_accuracy)
{
    // Propagate sigma points through measurement model
    Matrix<N_GNSS, N_SIGMA> Z_sigma;
    for (int i = 0; i < N_SIGMA; i++) {
        Z_sigma(0, i) = sigma(0, i); // px
        Z_sigma(1, i) = sigma(1, i); // py
        Z_sigma(2, i) = sigma(2, i); // pz
    }

    // Compute predicted measurement mean and covariance
    Matrix<N_GNSS,1> z_pred = weightedMean<N_GNSS>(Z_sigma);
    Matrix<N_GNSS,N_GNSS> S = weightedCovariance<N_GNSS, N_GNSS>(Z_sigma, z_pred, Z_sigma, z_pred);
    Matrix<N_STATE,N_GNSS> P_xz = weightedCovariance<N_STATE, N_GNSS>(sigmaMat, state, Z_sigma, z_pred);


    R_GNSS(0,0) = horizontal_accuracy * horizontal_accuracy;
    R_GNSS(1,1) = horizontal_accuracy * horizontal_accuracy;
    R_GNSS(2,2) = vertical_accuracy * vertical_accuracy;
    
    S += R_GNSS;
    
    // Kalman gain
    Matrix<N_STATE,N_GNSS> K = P_xz * S.inverse();

    // Update state and covariance
    Matrix<N_GNSS,1> y = z_meas - z_pred; // innovation
    state += K * y;
    P -= K * S * K.transpose();
}

void UKF::updateBarometer(float z_meas)
{
    // Propagate sigma points through measurement model
    Matrix<N_BARO, N_SIGMA> Z_sigma;
    for (int i = 0; i < N_SIGMA; i++) {
        Z_sigma(0, i) = sigma(2, i); // pz
    }

    // Compute predicted measurement mean and covariance
    Matrix<N_BARO,1> z_pred = weightedMean<N_BARO>(Z_sigma);
    Matrix<N_BARO,N_BARO> S = weightedCovariance<N_BARO, N_BARO>(Z_sigma, z_pred, Z_sigma, z_pred);
    Matrix<N_STATE,N_BARO> P_xz = weightedCovariance<N_STATE, N_BARO>(sigmaMat, state, Z_sigma, z_pred);

    S += R_Baro;
    
    // Kalman gain
    Matrix<N_STATE,N_BARO> K = P_xz * S.inverse();

    // Update state and covariance
    Matrix<N_BARO,1> y = z_meas - z_pred; // innovation
    state += K * y;
    P -= K * S * K.transpose();
}

void UKF::updateLidar(float z_meas)
{
    // Propagate sigma points through measurement model
    Matrix<N_LIDAR, N_SIGMA> Z_sigma;
    for (int i = 0; i < N_SIGMA; i++) {
        Z_sigma(0, i) = sigma(2, i); // pz
    }

    // Compute predicted measurement mean and covariance
    Matrix<N_LIDAR,1> z_pred = weightedMean<N_LIDAR>(Z_sigma);
    Matrix<N_LIDAR,N_LIDAR> S = weightedCovariance<N_LIDAR, N_LIDAR>(Z_sigma, z_pred, Z_sigma, z_pred);
    Matrix<N_STATE,N_LIDAR> P_xz = weightedCovariance<N_STATE, N_LIDAR>(sigmaMat, state, Z_sigma, z_pred);

    S += R_Baro;
    
    // Kalman gain
    Matrix<N_STATE,N_LIDAR> K = P_xz * S.inverse();

    // Update state and covariance
    Matrix<N_LIDAR,1> y = z_meas - z_pred; // innovation
    state += K * y;
    P -= K * S * K.transpose();
}
