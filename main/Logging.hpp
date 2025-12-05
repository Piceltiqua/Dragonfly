#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>

#include <cstdint>

#include "EKF.hpp"
#include "GNSS.hpp"
#include "IMU.hpp"

class Logging {
public:
    /**
     * Generate CSV header line for snapshot logging.
     * Returns a String containing all field names in the order they will be logged.
     */
    static String generateCsvHeader() {
        String header;
        // Original telemetry fields (preserved for backward compatibility)
        header += "time_us,";
        header += "posN_m,posE_m,posD_m,";
        header += "velN_m/s,velE_m/s,velD_m/s,";
        header += "Qw,Qx,Qy,Qz,Wx_rad/s,Wy_rad/s,Wz_rad/s,";
        header += "accN_m/s2,accE_m/s2,accD_m/s2,";
        header += "GNSS_lat_deg,GNSS_lon_deg,GNSS_alt_m,";
        header += "GNSS_posN_m,GNSS_posE_m,GNSS_posD_m,";
        header += "GNSS_velN_m/s,GNSS_velE_m/s,GNSS_velD_m/s,";
        header += "GNSS_HorAcc_m,GNSS_VertAcc_m,numSV,FixType,";
        header += "CurrentDraw_mA,CurrentConsumed_mAh,BatteryVoltage_V,BatteryLevel_percent,";
        header += "Motor1Throttle_percent,Motor2Throttle_percent,";
        header += "LegsPosition_servo,GimbalX_deg,GimbalY_deg,";

        // New intermediate variable fields
        // EKF predict fields
        header += ",EKF_pos_N,EKF_pos_E,EKF_pos_D";
        header += ",EKF_vel_N,EKF_vel_E,EKF_vel_D";
        header += ",EKF_F_0_0,EKF_F_0_1,EKF_F_0_2,EKF_F_0_3,EKF_F_0_4,EKF_F_0_5";
        header += ",EKF_F_1_1,EKF_F_1_2,EKF_F_1_3,EKF_F_1_4,EKF_F_1_5";
        header += ",EKF_F_2_2,EKF_F_2_3,EKF_F_2_4,EKF_F_2_5";
        header += ",EKF_F_3_3,EKF_F_3_4,EKF_F_3_5";
        header += ",EKF_F_4_4,EKF_F_4_5";
        header += ",EKF_F_5_5";

        header += ",EKF_Q_0_0,EKF_Q_0_1,EKF_Q_0_2,EKF_Q_0_3,EKF_Q_0_4,EKF_Q_0_5";
        header += ",EKF_Q_1_1,EKF_Q_1_2,EKF_Q_1_3,EKF_Q_1_4,EKF_Q_1_5";
        header += ",EKF_Q_2_2,EKF_Q_2_3,EKF_Q_2_4,EKF_Q_2_5";
        header += ",EKF_Q_3_3,EKF_Q_3_4,EKF_Q_3_5";
        header += ",EKF_Q_4_4,EKF_Q_4_5";
        header += ",EKF_Q_5_5";

        header += ",EKF_P_0_0,EKF_P_0_1,EKF_P_0_2,EKF_P_0_3,EKF_P_0_4,EKF_P_0_5";
        header += ",EKF_P_1_1,EKF_P_1_2,EKF_P_1_3,EKF_P_1_4,EKF_P_1_5";
        header += ",EKF_P_2_2,EKF_P_2_3,EKF_P_2_4,EKF_P_2_5";
        header += ",EKF_P_3_3,EKF_P_3_4,EKF_P_3_5";
        header += ",EKF_P_4_4,EKF_P_4_5";
        header += ",EKF_P_5_5";

        // EKF updateGNSS fields
        header += ",EKF_y_0,EKF_y_1,EKF_y_2,EKF_y_3,EKF_y_4,EKF_y_5";

        header += ",EKF_S_0_0,EKF_S_0_1,EKF_S_0_2,EKF_S_0_3,EKF_S_0_4,EKF_S_0_5";
        header += ",EKF_S_1_1,EKF_S_1_2,EKF_S_1_3,EKF_S_1_4,EKF_S_1_5";
        header += ",EKF_S_2_2,EKF_S_2_3,EKF_S_2_4,EKF_S_2_5";
        header += ",EKF_S_3_3,EKF_S_3_4,EKF_S_3_5";
        header += ",EKF_S_4_4,EKF_S_4_5";
        header += ",EKF_S_5_5";

        header += ",EKF_K_0_0,EKF_K_0_1,EKF_K_0_2,EKF_K_0_3,EKF_K_0_4,EKF_K_0_5";
        header += ",EKF_K_1_0,EKF_K_1_1,EKF_K_1_2,EKF_K_1_3,EKF_K_1_4,EKF_K_1_5";
        header += ",EKF_K_2_0,EKF_K_2_1,EKF_K_2_2,EKF_K_2_3,EKF_K_2_4,EKF_K_2_5";
        header += ",EKF_K_3_0,EKF_K_3_1,EKF_K_3_2,EKF_K_3_3,EKF_K_3_4,EKF_K_3_5";
        header += ",EKF_K_4_0,EKF_K_4_1,EKF_K_4_2,EKF_K_4_3,EKF_K_4_4,EKF_K_4_5";
        header += ",EKF_K_5_0,EKF_K_5_1,EKF_K_5_2,EKF_K_5_3,EKF_K_5_4,EKF_K_5_5";

        header += ",EKF_I_0_0,EKF_I_0_1,EKF_I_0_2,EKF_I_0_3,EKF_I_0_4,EKF_I_0_5";
        header += ",EKF_I_1_1,EKF_I_1_2,EKF_I_1_3,EKF_I_1_4,EKF_I_1_5";
        header += ",EKF_I_2_2,EKF_I_2_3,EKF_I_2_4,EKF_I_2_5";
        header += ",EKF_I_3_3,EKF_I_3_4,EKF_I_3_5";
        header += ",EKF_I_4_4,EKF_I_4_5";
        header += ",EKF_I_5_5";

        // GNSS position fields
        header += ",GNSS_r_ant_ned_x,GNSS_r_ant_ned_y,GNSS_r_ant_ned_z";
        header += ",GNSS_p_ant_ned_x,GNSS_p_ant_ned_y,GNSS_p_ant_ned_z";
        header += ",GNSS_p_cg_ned_x,GNSS_p_cg_ned_y,GNSS_p_cg_ned_z";

        // GNSS velocity fields
        header += ",GNSS_v_cg_ned_x,GNSS_v_cg_ned_y,GNSS_v_cg_ned_z";
        header += ",GNSS_v_ant_ned_x,GNSS_v_ant_ned_y,GNSS_v_ant_ned_z";
        header += ",GNSS_omega_ned_interp_x,GNSS_omega_ned_interp_y,GNSS_omega_ned_interp_z";
        header += ",GNSS_r_ant_ned_interp_x,GNSS_r_ant_ned_interp_y,GNSS_r_ant_ned_interp_z";

        // IMU fields
        header += ",IMU_q_imu_to_enu_w,IMU_q_imu_to_enu_x,IMU_q_imu_to_enu_y,IMU_q_imu_to_enu_z";
        header += ",IMU_q_cad_to_ned_w,IMU_q_cad_to_ned_x,IMU_q_cad_to_ned_y,IMU_q_cad_to_ned_z";
        header += ",IMU_acc_imu_x,IMU_acc_imu_y,IMU_acc_imu_z";
        header += ",IMU_acc_cad_x,IMU_acc_cad_y,IMU_acc_cad_z";
        header += ",IMU_acc_ned_x,IMU_acc_ned_y,IMU_acc_ned_z";
        header += ",IMU_omega_imu_x,IMU_omega_imu_y,IMU_omega_imu_z";

        return header;
    }

    /**
     * Pack all snapshot data into a CSV-formatted string using snprintf.
     * Returns the number of characters written (excluding null terminator).
     * Includes original telemetry fields first, then new intermediate variables.
     */
    static size_t packSnapshotCsv(char* buf, size_t bufSize,
                                  uint32_t time_us,
                                  const PosVel& posvel,
                                  const Attitude& attitude,
                                  const IMUAcceleration& imuAcc,
                                  const GNSSData& gnssData,
                                  const BatteryStatus& batteryStatus,
                                  const ActuatorCommands& actuators,
                                  const EkfSnapshot& ekf,
                                  const GnssSnapshot& gnss,
                                  const ImuSnapshot& imu) {
        int n = 0;
        int ret = 0;  // snprintf return value

        // Original telemetry fields
        ret = snprintf(buf + n, bufSize - n,
                       "%lu,"                                 // micros
                       "%.4f,%.4f,%.4f,"                      // posN,posE,posD
                       "%.4f,%.4f,%.4f,"                      // velN,velE,velD
                       "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"  // qw,qi,qj,qk,wx,wy,wz
                       "%.6f,%.6f,%.6f,"                      // imu ax,ay,az
                       "%.12f,%.12f,%.4f,"                    // GNSS lat(double), lon(double), alt(float)
                       "%.4f,%.4f,%.4f,"                      // GNSS posN,posE,posD
                       "%.4f,%.4f,%.4f,"                      // GNSS velN,velE,velD
                       "%.4f,%.4f,"                           // GNSS horAcc, vertAcc
                       "%u,%u,"                               // numSV (uint8_t), fixType (uint8_t)
                       "%u,%d,%u,%u,"                         // battery currentDraw, currentConsumed, batteryVoltage, batteryLevel
                       "%d,%d,%u,%d,%d,",                     // actuators motor1, motor2, legs, gimbalX, gimbalY
                       (unsigned long)time_us,
                       posvel.posN, posvel.posE, posvel.posD,
                       posvel.velN, posvel.velE, posvel.velD,
                       attitude.qw, attitude.qi, attitude.qj, attitude.qk,
                       attitude.wx, attitude.wy, attitude.wz,
                       imuAcc.ax_NED, imuAcc.ay_NED, imuAcc.az_NED,
                       gnssData.lat, gnssData.lon, gnssData.alt,
                       gnssData.posN, gnssData.posE, gnssData.posD,
                       gnssData.velN, gnssData.velE, gnssData.velD,
                       gnssData.horAcc, gnssData.vertAcc,
                       (unsigned int)gnssData.numSV,
                       (unsigned int)gnssData.fixType,
                       (unsigned int)batteryStatus.currentDraw,
                       (int)batteryStatus.currentConsumed,
                       (unsigned int)batteryStatus.batteryVoltage,
                       (unsigned int)batteryStatus.batteryLevel,
                       (int)actuators.motor1Throttle,
                       (int)actuators.motor2Throttle,
                       (unsigned int)actuators.legsPosition,
                       (int)actuators.servoXAngle,
                       (int)actuators.servoYAngle);
        if (ret < 0 || n + ret >= (int)bufSize) {
            Serial.print("[SNPRINTF_FAIL] Component=ORIG_TELEM ret=");
            Serial.println(ret);
            return 0;
        }
        n += ret;

        // New intermediate variable fields
        // EKF predict fields
        ret = snprintf(buf + n, bufSize - n, "%g,%g,%g,%g,%g,%g",
                       ekf.pos_N, ekf.pos_E, ekf.pos_D,
                       ekf.vel_N, ekf.vel_E, ekf.vel_D);
        if (ret < 0 || n + ret >= (int)bufSize) {
            Serial.print("[SNPRINTF_FAIL] Component=EKF_PREDICT ret=");
            Serial.println(ret);
            return 0;
        }
        n += ret;

        // EKF F upper triangle
        for (int i = 0; i < 21; ++i) {
            ret = snprintf(buf + n, bufSize - n, ",%g", ekf.F_upper[i]);
            if (ret < 0 || n + ret >= (int)bufSize) {
                Serial.print("[SNPRINTF_FAIL] Component=EKF_F[");
                Serial.print(i);
                Serial.print("] ret=");
                Serial.println(ret);
                return 0;
            }
            n += ret;
        }

        // EKF Q upper triangle
        for (int i = 0; i < 21; ++i) {
            ret = snprintf(buf + n, bufSize - n, ",%g", ekf.Q_upper[i]);
            if (ret < 0 || n + ret >= (int)bufSize) {
                Serial.print("[SNPRINTF_FAIL] Component=EKF_Q[");
                Serial.print(i);
                Serial.print("] ret=");
                Serial.println(ret);
                return 0;
            }
            n += ret;
        }

        // EKF P upper triangle
        for (int i = 0; i < 21; ++i) {
            ret = snprintf(buf + n, bufSize - n, ",%g", ekf.P_upper[i]);
            if (ret < 0 || n + ret >= (int)bufSize) {
                Serial.print("[SNPRINTF_FAIL] Component=EKF_P[");
                Serial.print(i);
                Serial.print("] ret=");
                Serial.println(ret);
                return 0;
            }
            n += ret;
        }

        // EKF y vector
        for (int i = 0; i < 6; ++i) {
            ret = snprintf(buf + n, bufSize - n, ",%g", ekf.y[i]);
            if (ret < 0 || n + ret >= (int)bufSize) {
                Serial.print("[SNPRINTF_FAIL] Component=EKF_y[");
                Serial.print(i);
                Serial.print("] ret=");
                Serial.println(ret);
                return 0;
            }
            n += ret;
        }

        // EKF S upper triangle
        for (int i = 0; i < 21; ++i) {
            ret = snprintf(buf + n, bufSize - n, ",%g", ekf.S_upper[i]);
            if (ret < 0 || n + ret >= (int)bufSize) {
                Serial.print("[SNPRINTF_FAIL] Component=EKF_S[");
                Serial.print(i);
                Serial.print("] ret=");
                Serial.println(ret);
                return 0;
            }
            n += ret;
        }

        // EKF K full matrix (6x6 = 36 elements)
        for (int i = 0; i < 36; ++i) {
            ret = snprintf(buf + n, bufSize - n, ",%g", ekf.K[i]);
            if (ret < 0 || n + ret >= (int)bufSize) {
                Serial.print("[SNPRINTF_FAIL] Component=EKF_K[");
                Serial.print(i);
                Serial.print("] ret=");
                Serial.println(ret);
                return 0;
            }
            n += ret;
        }

        // EKF I upper triangle
        for (int i = 0; i < 21; ++i) {
            ret = snprintf(buf + n, bufSize - n, ",%g", ekf.I_upper[i]);
            if (ret < 0 || n + ret >= (int)bufSize) {
                Serial.print("[SNPRINTF_FAIL] Component=EKF_I[");
                Serial.print(i);
                Serial.print("] ret=");
                Serial.println(ret);
                return 0;
            }
            n += ret;
        }

        // GNSS position fields
        ret = snprintf(buf + n, bufSize - n, ",%g,%g,%g,%g,%g,%g,%g,%g,%g",
                       gnss.r_ant_ned_x, gnss.r_ant_ned_y, gnss.r_ant_ned_z,
                       gnss.p_ant_ned_x, gnss.p_ant_ned_y, gnss.p_ant_ned_z,
                       gnss.p_cg_ned_x, gnss.p_cg_ned_y, gnss.p_cg_ned_z);
        if (ret < 0 || n + ret >= (int)bufSize) {
            Serial.print("[SNPRINTF_FAIL] Component=GNSS_POSITION ret=");
            Serial.println(ret);
            return 0;
        }
        n += ret;

        // GNSS velocity fields
        ret = snprintf(buf + n, bufSize - n, ",%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g",
                       gnss.v_cg_ned_x, gnss.v_cg_ned_y, gnss.v_cg_ned_z,
                       gnss.v_ant_ned_x, gnss.v_ant_ned_y, gnss.v_ant_ned_z,
                       gnss.omega_ned_interp_x, gnss.omega_ned_interp_y, gnss.omega_ned_interp_z,
                       gnss.r_ant_ned_interp_x, gnss.r_ant_ned_interp_y, gnss.r_ant_ned_interp_z);
        if (ret < 0 || n + ret >= (int)bufSize) {
            Serial.print("[SNPRINTF_FAIL] Component=GNSS_VELOCITY ret=");
            Serial.println(ret);
            return 0;
        }
        n += ret;

        // IMU fields
        ret = snprintf(buf + n, bufSize - n, ",%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g",
                       imu.q_imu_to_enu_w, imu.q_imu_to_enu_x, imu.q_imu_to_enu_y, imu.q_imu_to_enu_z,
                       imu.q_cad_to_ned_w, imu.q_cad_to_ned_x, imu.q_cad_to_ned_y, imu.q_cad_to_ned_z,
                       imu.acc_imu_x, imu.acc_imu_y, imu.acc_imu_z,
                       imu.acc_cad_x, imu.acc_cad_y, imu.acc_cad_z,
                       imu.acc_ned_x, imu.acc_ned_y, imu.acc_ned_z,
                       imu.omega_imu_x, imu.omega_imu_y, imu.omega_imu_z);
        if (ret < 0 || n + ret >= (int)bufSize) {
            Serial.print("[SNPRINTF_FAIL] Component=IMU ret=");
            Serial.println(ret);
            return 0;
        }
        n += ret;

        return n;
    }
};

#endif
