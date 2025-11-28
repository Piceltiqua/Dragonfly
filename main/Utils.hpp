#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include <eigen.h>
#include <stdint.h>
#include <string.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

static constexpr uint8_t STX = 0x7E;
static constexpr uint8_t DLE = 0x7D;
static constexpr uint8_t XOR_MASK = 0x20;

static constexpr uint8_t MSG_DATA = 0x01;
static constexpr uint8_t MSG_FLY = 0x06;
static constexpr uint8_t MSG_LEG = 0x1B;
static constexpr uint8_t MSG_BAT = 0x2C;
static constexpr uint8_t MSG_CTRL = 0x32;
static constexpr uint8_t MSG_ORIG = 0x57;
static constexpr uint8_t MSG_ENG = 0x58;
static constexpr uint8_t MSG_STOP = 0x60;
static constexpr uint8_t MSG_LAND = 0x6B;
static constexpr uint8_t MSG_REC = 0x8A;

static constexpr uint8_t LEGS_DEPLOYED = 0x9D;
static constexpr uint8_t LEGS_RETRACTED = 0xA9;
static constexpr uint8_t RECORDING_ON = 0xE6;
static constexpr uint8_t RECORDING_OFF = 0xF1;
static constexpr uint8_t CTRL_ATT_OFF_POS_OFF = 0xB7;
static constexpr uint8_t CTRL_ATT_ON_POS_OFF = 0xC3;
static constexpr uint8_t CTRL_ATT_ON_POS_ON = 0xCC;

// Fixed quaternion encoding the axis mapping between the CAD and the IMU
const Eigen::Quaternionf q_cad_to_imu = Eigen::Quaternionf(-0.5, -0.5, 0.5, 0.5);  // w,x,y,z
// Fixed quaternion that converts ENU to NED
const Eigen::Quaternionf q_enu_to_ned = Eigen::Quaternionf(0, 0.7071068, 0.7071068, 0);  // w,x,y,z

struct PosVel {
  float posN = 0.0f;
  float posE = 0.0f;
  float posD = 0.0f;
  float velN = 0.0f;
  float velE = 0.0f;
  float velD = 0.0f;
};

struct Attitude {
  float qw = 1.0f;
  float qi = 0.0f;
  float qj = 0.0f;
  float qk = 0.0f;
  float wx = 0.0f;
  float wy = 0.0f;
  float wz = 0.0f;
};

struct IMUAcceleration {
  float ax_NED = 0.0f;
  float ay_NED = 0.0f;
  float az_NED = 0.0f;
  uint8_t accuracy_status = 0;
};

struct GNSSData {
  double lat = 0.0f;
  double lon = 0.0f;
  float alt = 0.0f;
  double lat0 = 0.0f;
  double lon0 = 0.0f;
  float alt0 = 0.0f;
  float posN = 0.0f;
  float posE = 0.0f;
  float posD = 0.0f;
  float velN = 0.0f;
  float velE = 0.0f;
  float velD = 0.0f;
  float horAcc = 0.0f;
  float vertAcc = 0.0f;
  uint8_t numSV = 0;
  uint8_t fixType = 0;
};

struct BarometerData {
  float altBaro = 0.0f;
};

struct BatteryStatus {
  uint16_t currentDraw = 0;
  int16_t currentConsumed = 0;
  uint16_t batteryVoltage = 0;
  uint8_t batteryLevel = 0;
};

struct ActuatorCommands {
  int16_t motor1Throttle = 0;
  int16_t motor2Throttle = 0;
  uint8_t legsPosition = LEGS_DEPLOYED;  // 0xA9: retracted, 0x9D: deployed
  int16_t servoXAngle = 0;
  int16_t servoYAngle = 0;
};

#endif