#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>

#include <cstdint>

#include "GNSS.hpp"
#include "IMU.hpp"
#include "Utils.hpp"

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
        header += "Qw,Qx,Qy,Qz,Wx_rad/s,Wy_rad/s,Wz_rad/s,";
        header += "accN_m/s2,accE_m/s2,accD_m/s2,";
        header += "GNSS_lat_deg,GNSS_lon_deg,GNSS_alt_m,";
        header += "GNSS_posN_m,GNSS_posE_m,GNSS_posD_m,";
        header += "GNSS_velN_m/s,GNSS_velE_m/s,GNSS_velD_m/s,";
        header += "GNSS_HorAcc_m,GNSS_VertAcc_m,numSV,FixType,";
        header += "CurrentDraw_mA,CurrentConsumed_mAh,BatteryVoltage_mV,BatteryLevel_percent,";
        header += "MotorThrust_gram,";
        header += "LegsPosition_servo,ServoX_deg,ServoY_deg,GimbalX_deg,GimbalY_deg";

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

        // Position control setpoints
        header += ",PosSetpoint_N_m,PosSetpoint_E_m,PosSetpoint_D_m";
        header += ",VelSetpoint_N_m/s,VelSetpoint_E_m/s,VelSetpoint_D_m/s";

        // Attitude control setpoints
        header += ",AttSetpoint_pitch_deg,AttSetpoint_yaw_deg";

        // Position controller outputs
        header += ",MomentArm_m";

        // Position controller integrals
        header += ",PosIntegral_N_m*s,PosIntegral_E_m*s,PosIntegral_D_m*s";

        // Control errors
        header += ",Error_posN_m,Error_posE_m,Error_posD_m";
        header += ",Error_pitch_deg,Error_yaw_deg";

        return header;
    }

    /**
     * Pack all snapshot data into a CSV-formatted string using snprintf.
     * Returns the number of characters written (excluding null terminator).
     * Includes original telemetry fields first, then new intermediate variables.
     */
    static size_t packSnapshotCsv(char* buf, size_t bufSize,
                                  uint32_t time_us,
                                  const Attitude& attitude,
                                  const IMUAcceleration& imuAcc,
                                  const GNSSData& gnssData,
                                  const BatteryStatus& batteryStatus,
                                  const ActuatorCommands& actuators,
                                  const GnssSnapshot& gnss,
                                  const ImuSnapshot& imu,
                                  const PosCtrlSetpoint& posSetpoint,
                                  const PosCtrlOutput& posOutput,
                                  float N_integral,
                                  float E_integral,
                                  float D_integral,
                                  const ControlErrors& errors) {
        int n = 0;
        int ret = 0;  // snprintf return value

        // Original telemetry fields
        ret = snprintf(buf + n, bufSize - n,
                       "%lu,"                                 // micros
                       "%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,"  // qw,qi,qj,qk,wx,wy,wz
                       "%.6f,%.6f,%.6f,"                      // imu ax,ay,az
                       "%.12f,%.12f,%.4f,"                    // GNSS lat(double), lon(double), alt(float)
                       "%.4f,%.4f,%.4f,"                      // GNSS posN,posE,posD
                       "%.4f,%.4f,%.4f,"                      // GNSS velN,velE,velD
                       "%.4f,%.4f,"                           // GNSS horAcc, vertAcc
                       "%u,%u,"                               // numSV (uint8_t), fixType (uint8_t)
                       "%u,%d,%u,%u,"                         // battery currentDraw, currentConsumed, batteryVoltage, batteryLevel
                       "%d,%u,%d,%d,"                         // actuators motorThrust, legsPosition, servoX, servoY
                       "%.2f,%.2f",                           // gimbal angles
                       (unsigned long)time_us,
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
                       (int)actuators.motorThrust,
                       (unsigned int)actuators.legsPosition,
                       (int)actuators.servoXAngle,
                       (int)actuators.servoYAngle,
                       actuators.gimbalXAngle,
                       actuators.gimbalYAngle);
        if (ret < 0 || n + ret >= (int)bufSize) {
            Serial.print("[SNPRINTF_FAIL] Component=ORIG_TELEM ret=");
            Serial.println(ret);
            return 0;
        }
        n += ret;

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
        ret = snprintf(buf + n, bufSize - n, ",%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g",
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

        // Position control setpoints
        ret = snprintf(buf + n, bufSize - n, ",%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                       posSetpoint.posN, posSetpoint.posE, posSetpoint.posD,
                       posSetpoint.velN, posSetpoint.velE, posSetpoint.velD);
        if (ret < 0 || n + ret >= (int)bufSize) {
            Serial.print("[SNPRINTF_FAIL] Component=POS_SETPOINT ret=");
            Serial.println(ret);
            return 0;
        }
        n += ret;

        // Attitude control setpoints (pitch and yaw in degrees)
        float pitch_sp_deg = posOutput.attitudeSetpoint.pitch * 57.2957795f;  // rad to deg
        float yaw_sp_deg = posOutput.attitudeSetpoint.yaw * 57.2957795f;      // rad to deg
        ret = snprintf(buf + n, bufSize - n, ",%.2f,%.2f",
                       pitch_sp_deg, yaw_sp_deg);
        if (ret < 0 || n + ret >= (int)bufSize) {
            Serial.print("[SNPRINTF_FAIL] Component=ATT_SETPOINT ret=");
            Serial.println(ret);
            return 0;
        }
        n += ret;

        // Position controller outputs (moment arm)
        ret = snprintf(buf + n, bufSize - n, ",%.3f",
                       posOutput.momentArm);
        if (ret < 0 || n + ret >= (int)bufSize) {
            Serial.print("[SNPRINTF_FAIL] Component=POS_OUTPUT ret=");
            Serial.println(ret);
            return 0;
        }
        n += ret;

        // Position controller integrals
        ret = snprintf(buf + n, bufSize - n, ",%.2f,%.2f,%.2f",
                       N_integral, E_integral, D_integral);
        if (ret < 0 || n + ret >= (int)bufSize) {
            Serial.print("[SNPRINTF_FAIL] Component=POS_INTEGRAL ret=");
            Serial.println(ret);
            return 0;
        }
        n += ret;

        // Control errors
        ret = snprintf(buf + n, bufSize - n, ",%.3f,%.3f,%.3f,%.2f,%.2f",
                       errors.error_posN_m, errors.error_posE_m, errors.error_posD_m,
                       errors.error_pitch_deg, errors.error_yaw_deg);
        if (ret < 0 || n + ret >= (int)bufSize) {
            Serial.print("[SNPRINTF_FAIL] Component=ERRORS ret=");
            Serial.println(ret);
            return 0;
        }
        n += ret;

        return n;
    }
};

#endif
