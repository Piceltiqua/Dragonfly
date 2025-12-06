#include "EKF.hpp"

void EKF::setup() {
    state = MatrixXf::Zero(N_STATE, 1);

    float pos_var = 10.0f;  // (m^2)
    float vel_var = 1.0f;   // (m^2/s^2)

    P = Matrix<float, N_STATE, N_STATE>::Zero();
    P.block<3, 3>(0, 0) = Matrix3f::Identity() * pos_var;
    P.block<3, 3>(3, 3) = Matrix3f::Identity() * vel_var;
}

void EKF::predict(float dt) {
    // 1) Extract and update state using IMU acceleration
    state << posvel_.posN, posvel_.posE, posvel_.posD,
        posvel_.velN, posvel_.velE, posvel_.velD;

    Vector3f pos = state.block<3, 1>(0, 0);
    Vector3f vel = state.block<3, 1>(3, 0);

    Vector3f acc(imuAcc_.ax_NED,
                 imuAcc_.ay_NED,
                 imuAcc_.az_NED);

    // Kinematics:
    // vel_new = vel_old + a * dt
    // pos_new = pos_old + vel_old * dt + 0.5 * a * dt^2
    pos = pos + vel * dt + 0.5f * acc * dt * dt;
    vel = vel + acc * dt;

    state.block<3, 1>(0, 0) = pos;
    state.block<3, 1>(3, 0) = vel;

    // 2) Build state transition matrix F_mat
    Matrix<float, N_STATE, N_STATE> F_mat = Matrix<float, N_STATE, N_STATE>::Identity();
    F_mat.block<3, 3>(0, 3) = Matrix3f::Identity() * dt;

    // 3) Build process noise matrix Q
    MatrixXf Q = computeQ(dt);

    // 4) Propagate covariance
    //          P = F_mat P F_matᵀ + Q
    P = F_mat * P * F_mat.transpose() + Q;

    // 5) Update external output struct
    posvel_.posN = state(0, 0);
    posvel_.posE = state(1, 0);
    posvel_.posD = state(2, 0);
    posvel_.velN = state(3, 0);
    posvel_.velE = state(4, 0);
    posvel_.velD = state(5, 0);

    // Store snapshot for logging
    last_snapshot_.pos_N = pos(0);
    last_snapshot_.pos_E = pos(1);
    last_snapshot_.pos_D = pos(2);
    last_snapshot_.vel_N = vel(0);
    last_snapshot_.vel_E = vel(1);
    last_snapshot_.vel_D = vel(2);

    // Store upper triangle of F_mat (6x6)
    for (int i = 0, idx = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            last_snapshot_.F_upper[idx++] = F_mat(i, j);
        }
    }

    // Store upper triangle of Q (6x6)
    for (int i = 0, idx = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            last_snapshot_.Q_upper[idx++] = Q(i, j);
        }
    }

    // Store upper triangle of P (6x6)
    for (int i = 0, idx = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            last_snapshot_.P_upper[idx++] = P(i, j);
        }
    }
}

void EKF::updateGNSS() {
    Serial.println("\n===== EKF::updateGNSS() =====");
    Serial.print("CURRENT TIME : ");
    Serial.println(micros());

    // ------------------------------------------------------------
    // 1) Measurement vector z
    // ------------------------------------------------------------
    Matrix<float, N_GNSS, 1> z;
    z << gnssData_.posN,
        gnssData_.posE,
        gnssData_.posD,
        gnssData_.velN,
        gnssData_.velE,
        gnssData_.velD;

    Serial.println("state (before update):");
    for (int i = 0; i < N_GNSS; i++) {
        Serial.print(state(i, 0), 6);
        Serial.print(" ");
    }
    Serial.println("\n");

    Serial.println("z (measurement vector):");
    for (int i = 0; i < N_GNSS; i++) {
        Serial.print(z(i, 0), 6);
        Serial.print(" ");
    }
    Serial.println("\n");

    // ------------------------------------------------------------
    // 2) Measurement covariance R
    // ------------------------------------------------------------
    Matrix<float, N_GNSS, N_GNSS> R = Matrix<float, N_GNSS, N_GNSS>::Zero();

    R(0, 0) = gnssData_.horAcc * gnssData_.horAcc + s_r2;
    R(1, 1) = gnssData_.horAcc * gnssData_.horAcc + s_r2;
    R(2, 2) = gnssData_.vertAcc * gnssData_.vertAcc;
    R(3, 3) = s_v2;
    R(4, 4) = s_v2;
    R(5, 5) = s_v2;

    Serial.println("R (measurement covariance):");
    for (int r = 0; r < N_GNSS; r++) {
        for (int c = 0; c < N_GNSS; c++) {
            Serial.print(R(r, c), 6);
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println();

    // ------------------------------------------------------------
    // 3) Innovation y = z − state
    // ------------------------------------------------------------
    Serial.println("state (before update):");
    for (int r = 0; r < N_STATE; r++) {
        Serial.print(state(r, 0), 6);
        Serial.print(" ");
    }
    Serial.println("\n");

    Matrix<float, N_GNSS, 1> y = z - state;

    Serial.println("y (innovation):");
    for (int i = 0; i < N_GNSS; i++) {
        Serial.print(y(i, 0), 6);
        Serial.print(" ");
    }
    Serial.println("\n");

    // ------------------------------------------------------------
    // 4) Innovation covariance S = P + R
    // ------------------------------------------------------------
    Serial.println("P (before update):");
    for (int r = 0; r < N_STATE; r++) {
        for (int c = 0; c < N_STATE; c++) {
            Serial.print(P(r, c), 6);
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println();

    Matrix<float, N_GNSS, N_GNSS> S = P + R;

    Serial.println("S (innovation covariance):");
    for (int r = 0; r < N_GNSS; r++) {
        for (int c = 0; c < N_GNSS; c++) {
            Serial.print(S(r, c), 6);
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println();

    // ------------------------------------------------------------
    // 5) Kalman gain K = P * S.inverse()
    // ------------------------------------------------------------
    Matrix<float, N_STATE, N_GNSS> K = P * S.inverse();

    Serial.println("K (Kalman gain):");
    for (int r = 0; r < N_STATE; r++) {
        for (int c = 0; c < N_GNSS; c++) {
            Serial.print(K(r, c), 6);
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println();

    // ------------------------------------------------------------
    // 6) State update
    // ------------------------------------------------------------
    state = state + K * y;

    Serial.println("state (after update):");
    for (int r = 0; r < N_STATE; r++) {
        Serial.print(state(r, 0), 6);
        Serial.print(" ");
    }
    Serial.println("\n");

    // ------------------------------------------------------------
    // 7) Covariance update: P = (I − K) * P
    // ------------------------------------------------------------
    Matrix<float, N_STATE, N_STATE> I = Matrix<float, N_STATE, N_STATE>::Identity();
    P = (I - K) * P;

    Serial.println("P (after update):");
    for (int r = 0; r < N_STATE; r++) {
        for (int c = 0; c < N_STATE; c++) {
            Serial.print(P(r, c), 6);
            Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println();

    // ------------------------------------------------------------
    // 8) Export to posvel_
    // ------------------------------------------------------------
    posvel_.posN = state(0, 0);
    posvel_.posE = state(1, 0);
    posvel_.posD = state(2, 0);
    posvel_.velN = state(3, 0);
    posvel_.velE = state(4, 0);
    posvel_.velD = state(5, 0);

    Serial.println("===== END EKF::updateGNSS() =====\n");

    // Store snapshot for updateGNSS logging
    for (int i = 0; i < 6; ++i) {
        last_snapshot_.y[i] = y(i, 0);
    }

    // Store upper triangle of S (6x6)
    for (int i = 0, idx = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            last_snapshot_.S_upper[idx++] = S(i, j);
        }
    }

    // Store full K (6x6, not symmetric)
    for (int i = 0, idx = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            last_snapshot_.K[idx++] = K(i, j);
        }
    }

    // Store upper triangle of I (6x6)
    for (int i = 0, idx = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            last_snapshot_.I_upper[idx++] = I(i, j);
        }
    }
}

MatrixXf EKF::computeQ(float dt) {
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    // Discrete-time constant-acceleration model per axis:
    float q_pp = dt4 * 0.25f * s_a2;  // pos-pos
    float q_pv = dt3 * 0.5f * s_a2;   // pos-vel and vel-pos
    float q_vv = dt2 * s_a2;          // vel-vel

    MatrixXf Q = MatrixXf::Zero(N_STATE, N_STATE);
    Q.block<3, 3>(0, 0) = Matrix3f::Identity() * q_pp;
    Q.block<3, 3>(0, 3) = Matrix3f::Identity() * q_pv;
    Q.block<3, 3>(3, 0) = Matrix3f::Identity() * q_pv;
    Q.block<3, 3>(3, 3) = Matrix3f::Identity() * q_vv;

    return Q;
}