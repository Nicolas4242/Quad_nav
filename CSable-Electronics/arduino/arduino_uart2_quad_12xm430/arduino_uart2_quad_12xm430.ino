#include <Dynamixel2Arduino.h>
#include <math.h>
#include <ctype.h>
#include <string.h>


// ===========================
// Implémentation gyroscope
// ===========================
// --- AJOUT : Capteur BNO055 ---
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool bnoReady = false;

// Période d'envoi IMU vers la Raspberry Pi (20 ms = 50 Hz, idéal pour ROS)
const unsigned long IMU_PERIOD_MS = 20;
unsigned long lastImuMs = 0;
// ------------------------------

// =====================================================
// Arduino Mega 2560 + ROBOTIS DYNAMIXEL Shield
// Quadruped standard gait converted from MATLAB files:
//   - main_hardware_standard_gait.m
//   - inverse_kinematics.m
//   - forward_kinematics.m
//   - DynamixelInterface.m
//   - OpenLoopControl.m
//
// IMPORTANT UART RULE:
// - DYNAMIXEL bus uses Serial through the ROBOTIS Shield.
// - Raspberry Pi / external commands and status use UART2 = Serial2.
// - Arduino Mega pins:
//      TX2 = pin 16  -> connect to Raspberry Pi RX, through level shifting
//      RX2 = pin 17  -> connect to Raspberry Pi TX
// =====================================================

// ---------------- DYNAMIXEL bus ----------------
#define DXL_SERIAL   Serial
#define DXL_DIR_PIN  2
#define DXL_BAUDRATE 1000000UL

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// ---------------- Raspberry Pi command/status UART2 ----------------
#define CMD_SERIAL   Serial2
#define CMD_BAUDRATE 115200UL

// =====================================================
// General math constants
// =====================================================
const float PI_F         = 3.14159265358979323846f;
const float TWO_PI_F     = 2.0f * PI_F;
const float DEG_TO_RAD_F = PI_F / 180.0f;
const float RAD_TO_DEG_F = 180.0f / PI_F;

// =====================================================
// MATLAB gait parameters
// =====================================================
const float TS = 0.01f;                    // MATLAB Ts = 0.01, 100 Hz
const unsigned long TS_MS = 10;
const float FREQ = 2.0f;                   // MATLAB freq = 2.0 Hz
const int N_CYC = 50;                      // round((1/FREQ) / TS) = 50 samples

// Individual leg phase offsets from MATLAB.
const float PHASE_FR = 0.00f;
const float PHASE_FL = 0.50f;
const float PHASE_RR = 0.50f;
const float PHASE_RL = 0.00f;

const int SHIFT_FR = (int)(N_CYC * PHASE_FR + 0.5f);
const int SHIFT_FL = (int)(N_CYC * PHASE_FL + 0.5f);
const int SHIFT_RR = (int)(N_CYC * PHASE_RR + 0.5f);
const int SHIFT_RL = (int)(N_CYC * PHASE_RL + 0.5f);

// Cycloid trajectory from MATLAB.
const float XC = 9.0f;
const float YC = -150.0f;
const float AMP_X = 40.0f;                 // MATLAB A
const float AMP_Y = 20.0f;                 // MATLAB B

// Standing point from MATLAB.
const float STAND_X = 0.0f;
const float STAND_Y = -150.0f;

// Smooth standup from MATLAB section 4.5.
const float START_X = 0.0f;
const float START_Y = -160.0f;
const float STANDUP_DURATION = 3.0f;
const int STANDUP_STEPS = (int)(STANDUP_DURATION / TS + 0.5f);

// OpenLoopControl limiter from MATLAB: 240 deg/s.
const float MAX_SPEED_DEG_PER_SEC = 240.0f;
const float MAX_SPEED_RAD_PER_SEC = MAX_SPEED_DEG_PER_SEC * DEG_TO_RAD_F;
const float MAX_DELTA_ANGLE = MAX_SPEED_RAD_PER_SEC * TS;

// Periodic Arduino -> Raspberry Pi scan/status.
const unsigned long STATUS_PERIOD_MS = 5000;

// =====================================================
// Motor types and configuration
// =====================================================
enum MotorType {
  MOTOR_XM430_W350_R
};

struct MotorConfig {
  const char* name;
  uint8_t id;
  MotorType type;
  float protocol;
  float maxDegree;
  int32_t maxRaw;
  int32_t tolerance;
  int direction;
  float offsetRad;
};

// Order of X-Series leg motors matches MATLAB hw_params.DXL_IDS:
// [FR thigh, FR crank, FL thigh, FL crank, RR thigh, RR crank, RL thigh, RL crank]
// Z joint order: [FL-Z, FR-Z, RL-Z, RR-Z]. Replacement IDs are NOT assigned.
enum MotorIndex {
  FR_THIGH = 0,
  FR_CRANK = 1,
  FL_THIGH = 2,
  FL_CRANK = 3,
  RR_THIGH = 4,
  RR_CRANK = 5,
  RL_THIGH = 6,
  RL_CRANK = 7,
  FL_Z  = 8,
  FR_Z  = 9,
  RL_Z  = 10,
  RR_Z  = 11
};

const uint8_t UNCONFIGURED_MOTOR_ID = 255; // Invalid ID; never send to it.

MotorConfig motors[] = {
  // name,       id, type,              proto, maxDeg, maxRaw, tol, direction, offsetRad
  {"FR_THIGH",  1,  MOTOR_XM430_W350_R, 2.0f, 360.0f, 4095, 30,  1,   0.0f * DEG_TO_RAD_F},
  {"FR_CRANK",  2,  MOTOR_XM430_W350_R, 2.0f, 360.0f, 4095, 30,  1,   0.0f * DEG_TO_RAD_F},

  {"FL_THIGH",  3,  MOTOR_XM430_W350_R, 2.0f, 360.0f, 4095, 30, -1,   0.0f * DEG_TO_RAD_F},
  {"FL_CRANK",  4,  MOTOR_XM430_W350_R, 2.0f, 360.0f, 4095, 30, -1, -17.0f * DEG_TO_RAD_F},

  {"RR_THIGH",  7,  MOTOR_XM430_W350_R, 2.0f, 360.0f, 4095, 30,  1,   0.0f * DEG_TO_RAD_F},
  {"RR_CRANK",  8,  MOTOR_XM430_W350_R, 2.0f, 360.0f, 4095, 30,  1,   0.0f * DEG_TO_RAD_F},

  {"RL_THIGH",  5,  MOTOR_XM430_W350_R, 2.0f, 360.0f, 4095, 30, -1,   0.0f * DEG_TO_RAD_F},
  {"RL_CRANK",  6,  MOTOR_XM430_W350_R, 2.0f, 360.0f, 4095, 30, -1,   0.0f * DEG_TO_RAD_F},

  // TODO(calibration): Replace only physically verified IDs, hold tolerances (raw),
  // directions (+1/-1), and software offsets (rad). Sentinels intentionally block motion.
  {"FL_Z", UNCONFIGURED_MOTOR_ID, MOTOR_XM430_W350_R, 2.0f, 360.0f, 4095, -1, 0, NAN},
  {"FR_Z", UNCONFIGURED_MOTOR_ID, MOTOR_XM430_W350_R, 2.0f, 360.0f, 4095, -1, 0, NAN},
  {"RL_Z", UNCONFIGURED_MOTOR_ID, MOTOR_XM430_W350_R, 2.0f, 360.0f, 4095, -1, 0, NAN},
  {"RR_Z", UNCONFIGURED_MOTOR_ID, MOTOR_XM430_W350_R, 2.0f, 360.0f, 4095, -1, 0, NAN}
};

const int LEG_MOTOR_COUNT = 8;
const int Z_MOTOR_COUNT = 4;
const int ALL_MOTOR_COUNT = sizeof(motors) / sizeof(motors[0]);

const int Z_INDICES[Z_MOTOR_COUNT] = {FL_Z, FR_Z, RL_Z, RR_Z};

struct ZAxisCalibration {
  bool verified;                 // Deliberate physical sign-off; see README.
  float neutralJointRad;        // MODEL joint angle, not absolute motor angle.
  int32_t minRaw;               // Calibrated mechanical bounds in absolute motor counts.
  int32_t maxRaw;               // Must match the provisioned XM430 position limits.
  int32_t startupToleranceRaw;  // Maximum approved correction from manually placed neutral.
  int32_t profileVelocityRaw;   // Velocity-based profile, strictly positive.
  int32_t profileAccelerationRaw; // Strictly positive; zero is unlimited.
  int32_t expectedDriveMode;    // Only 0/1: velocity profile, no torque-on-goal-update.
  int32_t expectedHomingOffsetRaw; // INTERNAL offset; distinct from MotorConfig.offsetRad.
  int32_t expectedFirmwareVersion;
  int32_t expectedPwmLimitRaw;
  int32_t expectedCurrentLimitRaw;
  int32_t expectedVelocityLimitRaw;
};

// TODO(calibration): Fill every field from bench measurements and verified internal
// settings, then set verified=true for EACH joint. No motion-capable defaults.
const ZAxisCalibration zCalibration[Z_MOTOR_COUNT] = {
  {false, NAN, -1, -1, -1, -1, -1, -1, INT32_MIN, -1, -1, -1, -1}, // FL_Z
  {false, NAN, -1, -1, -1, -1, -1, -1, INT32_MIN, -1, -1, -1, -1}, // FR_Z
  {false, NAN, -1, -1, -1, -1, -1, -1, INT32_MIN, -1, -1, -1, -1}, // RL_Z
  {false, NAN, -1, -1, -1, -1, -1, -1, INT32_MIN, -1, -1, -1, -1}  // RR_Z
};
const unsigned long Z_SETTLE_TIMEOUT_MS = 5000;
const unsigned long Z_SETTLE_DWELL_MS = 200;

struct LegStartupCalibration {
  bool verified; // Mechanical safe interval and complete startup corridor checked on robot.
  int32_t minRaw;
  int32_t maxRaw;
  int32_t startToleranceRaw; // Approved initial deviation from the existing START_X/Y pose.
  int32_t holdToleranceRaw;  // Approved change at torque enable / measured-position hold.
  int32_t expectedDriveMode;
  int32_t expectedHomingOffsetRaw;
};
// TODO(commissioning): No mechanical ranges or acceptable starting errors are known here.
// These entries supplement, and never replace, the original eight MotorConfig mappings.
const LegStartupCalibration legStartup[LEG_MOTOR_COUNT] = {
  {false, -1, -1, -1, -1, -1, INT32_MIN}, // FR_THIGH
  {false, -1, -1, -1, -1, -1, INT32_MIN}, // FR_CRANK
  {false, -1, -1, -1, -1, -1, INT32_MIN}, // FL_THIGH
  {false, -1, -1, -1, -1, -1, INT32_MIN}, // FL_CRANK
  {false, -1, -1, -1, -1, -1, INT32_MIN}, // RR_THIGH
  {false, -1, -1, -1, -1, -1, INT32_MIN}, // RR_CRANK
  {false, -1, -1, -1, -1, -1, INT32_MIN}, // RL_THIGH
  {false, -1, -1, -1, -1, -1, INT32_MIN}  // RL_CRANK
};
// Physical sign-off must cover support against collapse on torque-off, the starting
// posture/collision corridor, bus timing and the supervision/fault response below.
// Not a calibration bypass: ALL motor checks must also pass. No serial unlock exists.
const bool MOTION_COMMISSIONING_VERIFIED = false;
const uint32_t DXL_IO_TIMEOUT_MS = 2;
const unsigned long Z_MONITOR_PERIOD_MS = 10; // One motor per slot; nominal full round 40 ms.
const unsigned long Z_MONITOR_MAX_AGE_MS = 200;
const unsigned long Z_MONITOR_BUDGET_US = 3000;

// =====================================================
// Kinematic constants from inverse_kinematics.m and forward_kinematics.m
// =====================================================
const float L2_DG = 100.00f;    // Thigh
const float L3_GH = 105.73f;    // Shin

const float AD = 41.00f;
const float AB = 20.10f;
const float BC = 29.49f;
const float CD = 28.07f;
const float DE = 27.94f;
const float CE = 38.18f;
const float EF = 100.00f;
const float FG = 27.27f;

const int BRANCH_KNEE  = -1;
const int BRANCH_E     = -1;
const int BRANCH_CRANK = 1;
const float A_OFFSET_RAD = 90.0f * DEG_TO_RAD_F;

// =====================================================
// Runtime state
// =====================================================
enum RunMode {
  MODE_STANDING,
  MODE_WALKING,
  MODE_STOPPED
};

enum LegIndex {
  LEG_FR = 0,
  LEG_FL = 1,
  LEG_RR = 2,
  LEG_RL = 3
};

RunMode mode = MODE_STANDING;

// previousAction[leg][joint]: joint 0 = thigh/theta2, joint 1 = crank/a
float previousAction[4][2] = {
  {0.0f, 0.0f},
  {0.0f, 0.0f},
  {0.0f, 0.0f},
  {0.0f, 0.0f}
};

float lastRef[4][2] = {
  {STAND_X, STAND_Y},
  {STAND_X, STAND_Y},
  {STAND_X, STAND_Y},
  {STAND_X, STAND_Y}
};

int phaseIdx = 0;
unsigned long nextLoopMs = 0;
unsigned long lastStatusMs = 0;
bool hardwareReady = false;
bool torqueEnabled = false;
bool zCalibrationReady = false;
bool zAxisReady = false;
bool torqueOffUnconfirmed = true; // Bus requests cannot prove power removal.
int32_t zNeutralRaw[Z_MOTOR_COUNT];
bool legStartupReady = false;
int32_t legStartRaw[LEG_MOTOR_COUNT];
int32_t legHoldRaw[LEG_MOTOR_COUNT];
unsigned long zLastGoodMs[Z_MOTOR_COUNT];
unsigned long nextZMonitorMs = 0;
uint8_t nextZMonitor = 0;
int faultMotorIndex = -1;
bool torqueOffDispatchAcknowledged = false;

char cmdBuffer[64];
uint8_t cmdLen = 0;

// Forward declaration because smoothStandupBlocking() keeps UART2 active.
void pollCommandSerial();
void safeCleanup();
bool readZItem(int z, uint8_t item, int32_t &value);
bool readMotorItem(int motorIndex, uint8_t item, int32_t &value);
bool dxlExchangeOK();
bool inverseKinematics(float x, float y, float q[2]);
void stopForFault(const char* reason);
bool revalidateZReady();
bool zFeedbackFresh();
void serviceZMonitor(unsigned long deadlineMs);

// =====================================================
// Utility functions
// =====================================================
float clampFloat(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float normalizeRadPositive(float a) {
  while (a < 0.0f) a += TWO_PI_F;
  while (a >= TWO_PI_F) a -= TWO_PI_F;
  return a;
}

float unwrapNear(float angle, float reference) {
  while ((angle - reference) > PI_F) angle -= TWO_PI_F;
  while ((angle - reference) < -PI_F) angle += TWO_PI_F;
  return angle;
}

bool isInvalid(float v) {
  return isnan(v) || isinf(v);
}

const char* modeName() {
  switch (mode) {
    case MODE_STANDING: return "STANDING";
    case MODE_WALKING:  return "WALKING";
    case MODE_STOPPED:  return "STOPPED";
    default:            return "UNKNOWN";
  }
}

const char* legName(int leg) {
  switch (leg) {
    case LEG_FR: return "fr";
    case LEG_FL: return "fl";
    case LEG_RR: return "rr";
    case LEG_RL: return "rl";
    default:     return "unknown";
  }
}

// =====================================================
// UART2 reporting: Arduino -> Raspberry Pi
// =====================================================
void sendLine(const char* msg) {
  CMD_SERIAL.println(msg);
}

void sendModeState(const char* reason) {
  CMD_SERIAL.print("STATE,");
  CMD_SERIAL.print(modeName());
  CMD_SERIAL.print(",reason=");
  CMD_SERIAL.println(reason);
}

void printLegStatusField(int leg) {
  const char* n = legName(leg);

  CMD_SERIAL.print(",");
  CMD_SERIAL.print(n);
  CMD_SERIAL.print("_ref_x=");
  CMD_SERIAL.print(lastRef[leg][0], 2);

  CMD_SERIAL.print(",");
  CMD_SERIAL.print(n);
  CMD_SERIAL.print("_ref_y=");
  CMD_SERIAL.print(lastRef[leg][1], 2);

  CMD_SERIAL.print(",");
  CMD_SERIAL.print(n);
  CMD_SERIAL.print("_q_deg=");
  CMD_SERIAL.print(previousAction[leg][0] * RAD_TO_DEG_F, 2);
  CMD_SERIAL.print("/");
  CMD_SERIAL.print(previousAction[leg][1] * RAD_TO_DEG_F, 2);
}

void sendStatus(bool includeMotorRead) {
  CMD_SERIAL.print("STATUS,mode=");
  CMD_SERIAL.print(modeName());
  CMD_SERIAL.print(",ready=");
  CMD_SERIAL.print(hardwareReady ? 1 : 0);
  CMD_SERIAL.print(",torque=");
  CMD_SERIAL.print(torqueEnabled ? 1 : 0);
  CMD_SERIAL.print(F(",z_calibrated="));
  CMD_SERIAL.print(zCalibrationReady ? 1 : 0);
  CMD_SERIAL.print(F(",z_ready="));
  CMD_SERIAL.print(zAxisReady ? 1 : 0);
  CMD_SERIAL.print(F(",torque_off_unconfirmed="));
  CMD_SERIAL.print(torqueOffUnconfirmed ? 1 : 0);
  CMD_SERIAL.print(",phase=");
  CMD_SERIAL.print(phaseIdx);
  CMD_SERIAL.print(",ncyc=");
  CMD_SERIAL.print(N_CYC);

  printLegStatusField(LEG_FR);
  printLegStatusField(LEG_FL);
  printLegStatusField(LEG_RR);
  printLegStatusField(LEG_RL);

  if (includeMotorRead && hardwareReady) {
    CMD_SERIAL.print(",raw_leg=");
    for (int i = 0; i < LEG_MOTOR_COUNT; i++) {
      int32_t raw;
      bool ok = readMotorItem(i, PRESENT_POSITION, raw);
      if (!ok) { faultMotorIndex = i; stopForFault("status_read_failed"); return; }
      CMD_SERIAL.print(motors[i].id);
      CMD_SERIAL.print(":");
      if (ok) CMD_SERIAL.print(raw);
      else CMD_SERIAL.print(F("ERR"));
      if (i < LEG_MOTOR_COUNT - 1) CMD_SERIAL.print("/");
    }

    CMD_SERIAL.print(",raw_z=");
    for (int j = 0; j < Z_MOTOR_COUNT; j++) {
      int idx = Z_INDICES[j];
      int32_t raw;
      bool ok = readMotorItem(idx, PRESENT_POSITION, raw);
      if (!ok) { faultMotorIndex = idx; stopForFault("status_read_failed"); return; }
      CMD_SERIAL.print(motors[idx].id);
      CMD_SERIAL.print(":");
      if (ok) CMD_SERIAL.print(raw);
      else CMD_SERIAL.print(F("ERR"));
      if (j < Z_MOTOR_COUNT - 1) CMD_SERIAL.print("/");
    }
  }

  CMD_SERIAL.println();
}

// =====================================================
// Protocol and conversion functions
// =====================================================
void selectMotorProtocol(int motorIndex) {
  dxl.setPortProtocolVersion(motors[motorIndex].protocol);
}

int32_t motorAbsRadToRaw(int motorIndex, float motorAbsRad) {
  (void)motorIndex; // Retain the baseline helper signature; all motors now use XM encoding.
  // Preserve the eight walking motors' encoding. Z uses zJointRadToRawChecked().
  motorAbsRad = normalizeRadPositive(motorAbsRad);
  int32_t raw = (int32_t)roundf(motorAbsRad * (4096.0f / TWO_PI_F));
  raw = (int32_t)clampFloat((float)raw, 0.0f, 4095.0f);
  return raw;
}

float motorRawToAbsRad(int motorIndex, int32_t raw) {
  (void)motorIndex;
  raw = (int32_t)clampFloat((float)raw, 0.0f, 4095.0f);
  return ((float)raw) * (TWO_PI_F / 4096.0f);
}

int32_t jointRadToMotorRaw(int motorIndex, float modelRad) {
  MotorConfig m = motors[motorIndex];
  float motorAbsRad = m.offsetRad + ((float)m.direction * modelRad);
  return motorAbsRadToRaw(motorIndex, motorAbsRad);
}

float motorRawToJointRad(int motorIndex, int32_t raw, float referenceModelRad) {
  MotorConfig m = motors[motorIndex];
  float motorAbsRad = motorRawToAbsRad(motorIndex, raw);
  float modelRad = (motorAbsRad - m.offsetRad) / (float)m.direction;
  return unwrapNear(modelRad, referenceModelRad);
}

// =====================================================
// Motor setup and command functions
// =====================================================
bool configuredMotorId(uint8_t id) {
  return id <= 252; // Exclude reserved IDs, broadcast (254), and unset (255).
}

bool dxlExchangeOK() {
  return dxl.getLastLibErrCode() == 0 && dxl.getLastStatusPacketError() == 0;
}

bool zFailure(int z, const __FlashStringHelper* reason) {
  faultMotorIndex = Z_INDICES[z];
  zAxisReady = false;
  // Dispatch shutdown before serial diagnostics can block on a full TX buffer.
  mode = MODE_STOPPED;
  safeCleanup();
  CMD_SERIAL.print(F("ERR,z_axis,joint="));
  CMD_SERIAL.print(motors[Z_INDICES[z]].name);
  CMD_SERIAL.print(F(",reason="));
  CMD_SERIAL.println(reason);
  return false;
}

bool zJointRadToRawChecked(int z, float modelRad, int32_t &raw) {
  const MotorConfig &m = motors[Z_INDICES[z]];
  const ZAxisCalibration &c = zCalibration[z];
  if (isInvalid(modelRad) || isInvalid(m.offsetRad) ||
      (m.direction != 1 && m.direction != -1)) return false;
  float motorAbsRad = m.offsetRad + ((float)m.direction * modelRad);
  // Z limits describe one continuous single-turn interval. Never wrap or clamp.
  float counts = motorAbsRad * (4096.0f / TWO_PI_F);
  if (isInvalid(counts) || counts < 0.0f || counts > 4095.0f) return false;
  raw = (int32_t)roundf(counts);
  return c.minRaw >= 0 && c.maxRaw <= 4095 && c.minRaw < c.maxRaw &&
         raw >= c.minRaw && raw <= c.maxRaw;
}

bool validateZCalibration() {
  for (int z = 0; z < Z_MOTOR_COUNT; z++) {
    const MotorConfig &m = motors[Z_INDICES[z]];
    const ZAxisCalibration &c = zCalibration[z];
    if (!c.verified || !configuredMotorId(m.id) ||
        m.type != MOTOR_XM430_W350_R || m.protocol != 2.0f ||
        m.maxRaw != 4095 || m.maxDegree != 360.0f ||
        !zJointRadToRawChecked(z, c.neutralJointRad, zNeutralRaw[z]) ||
        c.startupToleranceRaw < 0 || c.startupToleranceRaw > c.maxRaw - c.minRaw ||
        m.tolerance < 0 || m.tolerance > c.startupToleranceRaw ||
        c.profileVelocityRaw <= 0 || c.profileVelocityRaw > 32767 ||
        c.profileAccelerationRaw <= 0 || c.profileAccelerationRaw > 32767 ||
        (c.expectedDriveMode != 0 && c.expectedDriveMode != 1) ||
        c.expectedHomingOffsetRaw < -1024 || c.expectedHomingOffsetRaw > 1024 ||
        c.expectedFirmwareVersion <= 0 || c.expectedFirmwareVersion > 255 ||
        c.expectedPwmLimitRaw <= 0 || c.expectedPwmLimitRaw > 885 ||
        c.expectedCurrentLimitRaw <= 0 || c.expectedCurrentLimitRaw > 1193 ||
        c.expectedVelocityLimitRaw <= 0 || c.expectedVelocityLimitRaw > 1023 ||
        c.profileVelocityRaw > c.expectedVelocityLimitRaw) {
      return zFailure(z, F("calibration_missing_or_invalid"));
    }
    for (int i = 0; i < ALL_MOTOR_COUNT; i++) {
      if (i != Z_INDICES[z] && motors[i].id == m.id)
        return zFailure(z, F("duplicate_id"));
    }
  }
  return true;
}

bool readMotorItem(int motorIndex, uint8_t item, int32_t &value) {
  if (!configuredMotorId(motors[motorIndex].id)) return false;
  selectMotorProtocol(motorIndex);
  DYNAMIXEL::ControlTableItemInfo_t info = DYNAMIXEL::getControlTableItemInfo(XM430_W350, item);
  if (item == STARTUP_CONFIGURATION || item == BUS_WATCHDOG) {
    info.addr = (item == STARTUP_CONFIGURATION) ? 60 : 98;
    info.addr_length = 1; // Correct the library 0.8.2 XM430 table entries.
  }
  if (info.addr_length == 0 || info.addr_length > 4) return false;
  uint8_t bytes[4] = {0, 0, 0, 0};
  if (dxl.read(motors[motorIndex].id, info.addr, info.addr_length, bytes, sizeof(bytes),
               DXL_IO_TIMEOUT_MS) != info.addr_length || !dxlExchangeOK()) return false;
  // Unlike readControlTableItem(), keep unsigned 1/2-byte registers unsigned.
  // Four-byte position/velocity/homing values retain their signed bit pattern.
  uint32_t bits = (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
                  ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
  memcpy(&value, &bits, sizeof(value));
  return true;
}

bool expectMotorItem(int motorIndex, uint8_t item, int32_t expected) {
  int32_t actual;
  return readMotorItem(motorIndex, item, actual) && actual == expected;
}

bool writeMotorItem(int motorIndex, uint8_t item, int32_t value) {
  if (!zCalibrationReady || !legStartupReady || !MOTION_COMMISSIONING_VERIFIED ||
      mode == MODE_STOPPED || !configuredMotorId(motors[motorIndex].id)) return false;
  DYNAMIXEL::ControlTableItemInfo_t info = DYNAMIXEL::getControlTableItemInfo(XM430_W350, item);
  if (info.addr_length == 0 || info.addr_length > 4) return false;
  uint32_t bits = (uint32_t)value;
  uint8_t bytes[4] = {(uint8_t)bits, (uint8_t)(bits >> 8), (uint8_t)(bits >> 16), (uint8_t)(bits >> 24)};
  selectMotorProtocol(motorIndex);
  return dxl.write(motors[motorIndex].id, info.addr, bytes, info.addr_length, DXL_IO_TIMEOUT_MS) && dxlExchangeOK();
}

bool readZItem(int z, uint8_t item, int32_t &value) {
  if (!zCalibrationReady) return false;
  if (readMotorItem(Z_INDICES[z], item, value)) return true;
  return zFailure(z, F("read_failed"));
}

bool expectZItem(int z, uint8_t item, int32_t expected) {
  int32_t actual;
  if (!readZItem(z, item, actual)) return false;
  if (actual == expected) return true;
  zFailure(z, F("setting_mismatch"));
  CMD_SERIAL.print(F("ERR,z_setting,item="));
  CMD_SERIAL.print(item);
  CMD_SERIAL.print(F(",expected="));
  CMD_SERIAL.print(expected);
  CMD_SERIAL.print(F(",actual="));
  CMD_SERIAL.println(actual);
  return false;
}

bool writeZItemChecked(int z, uint8_t item, int32_t value) {
  if (!zCalibrationReady || mode == MODE_STOPPED ||
      !configuredMotorId(motors[Z_INDICES[z]].id)) return false;
  if (!writeMotorItem(Z_INDICES[z], item, value))
    return zFailure(z, F("write_failed"));
  return expectZItem(z, item, value);
}

bool readZStartupPosition(int z, int32_t &raw) {
  const ZAxisCalibration &c = zCalibration[z];
  int32_t velocity;
  if (!readZItem(z, PRESENT_POSITION, raw) || !readZItem(z, PRESENT_VELOCITY, velocity)) return false;
  // Torque-on resets the position counter in position mode. Reject ambiguous turns
  // instead of normalizing, including a turn hidden by the internal homing offset.
  if (raw < c.minRaw || raw > c.maxRaw ||
      raw - c.expectedHomingOffsetRaw < 0 || raw - c.expectedHomingOffsetRaw > 4095 ||
      labs(raw - zNeutralRaw[z]) > c.startupToleranceRaw || velocity != 0)
    return zFailure(z, F("manually_position_near_neutral_and_stop"));
  return true;
}

bool prepareZMotor(int z) {
  const ZAxisCalibration &c = zCalibration[z];
  // All Z joints have already received torque-off before any preparation.
  // Provision EEPROM during calibration. Reject mismatches instead of rewriting
  // direction, homing or mode here (mode changes can reset gains/position).
  if (!expectZItem(z, MODEL_NUMBER, XM430_W350) ||
      !expectZItem(z, FIRMWARE_VERSION, c.expectedFirmwareVersion) ||
      !expectZItem(z, TORQUE_ENABLE, 0) ||
      !expectZItem(z, OPERATING_MODE, 3) || // XM430 Position Control Mode (OP_POSITION).
      !expectZItem(z, DRIVE_MODE, c.expectedDriveMode) ||
      !expectZItem(z, HOMING_OFFSET, c.expectedHomingOffsetRaw) ||
      !expectZItem(z, MIN_POSITION_LIMIT, c.minRaw) ||
      !expectZItem(z, MAX_POSITION_LIMIT, c.maxRaw) ||
      !expectZItem(z, PWM_LIMIT, c.expectedPwmLimitRaw) ||
      !expectZItem(z, CURRENT_LIMIT, c.expectedCurrentLimitRaw) ||
      !expectZItem(z, VELOCITY_LIMIT, c.expectedVelocityLimitRaw) ||
      !expectZItem(z, STATUS_RETURN_LEVEL, 2) ||
      !expectZItem(z, REGISTERED_INSTRUCTION, 0) ||
      !expectZItem(z, HARDWARE_ERROR_STATUS, 0) ||
      !expectZItem(z, BUS_WATCHDOG, 0)) return false;
  int32_t secondaryId;
  if (!readZItem(z, SECONDARY_ID, secondaryId)) return false;
  if (secondaryId < 253) return zFailure(z, F("secondary_id_must_be_disabled"));
  if (c.expectedFirmwareVersion >= 45 && !expectZItem(z, STARTUP_CONFIGURATION, 0)) return false;
  if (!writeZItemChecked(z, PROFILE_VELOCITY, c.profileVelocityRaw) ||
      !writeZItemChecked(z, PROFILE_ACCELERATION, c.profileAccelerationRaw)) return false;
  int32_t raw;
  return readZStartupPosition(z, raw);
}

bool zSettleObservation(bool settled, unsigned long observedMs, unsigned long startMs,
                        bool &observingStable, unsigned long &stableSinceMs) {
  if (!settled || observedMs - startMs >= Z_SETTLE_TIMEOUT_MS) {
    observingStable = false;
    return false;
  }
  if (!observingStable) {
    stableSinceMs = observedMs;
    observingStable = true;
    return false;
  }
  return observedMs - stableSinceMs >= Z_SETTLE_DWELL_MS;
}

bool holdZAxis() {
  if (!zCalibrationReady || !legStartupReady || !MOTION_COMMISSIONING_VERIFIED || mode == MODE_STOPPED) return false;
  // Preflight ALL four joints with torque off before enabling any of them.
  for (int z = 0; z < Z_MOTOR_COUNT; z++) {
    pollCommandSerial();
    if (mode == MODE_STOPPED || !prepareZMotor(z)) return false;
  }
  for (int z = 0; z < Z_MOTOR_COUNT; z++) {
    pollCommandSerial();
    if (mode == MODE_STOPPED) return false;
    int32_t raw;
    if (!readZStartupPosition(z, raw) ||
        !writeZItemChecked(z, GOAL_POSITION, raw) || // Seed measured absolute position.
        !expectZItem(z, TORQUE_ENABLE, 0)) return false;
    torqueOffUnconfirmed = true;
    if (!writeZItemChecked(z, TORQUE_ENABLE, 1)) return false;
    int32_t enabledRaw;
    if (!readZStartupPosition(z, enabledRaw) ||
        labs(enabledRaw - raw) > motors[Z_INDICES[z]].tolerance)
      return zFailure(z, F("position_changed_on_torque_enable"));
    // Only the calibrated, bounded near-neutral correction is permitted.
    if (!writeZItemChecked(z, GOAL_POSITION, zNeutralRaw[z])) return false;
  }
  unsigned long startMs = millis();
  unsigned long stableSinceMs = 0;
  bool observingStable = false;
  while (millis() - startMs < Z_SETTLE_TIMEOUT_MS) {
    pollCommandSerial();
    if (mode == MODE_STOPPED) return false;
    bool settled = true;
    for (int z = 0; z < Z_MOTOR_COUNT; z++) {
      int32_t raw, moving;
      if (!readZItem(z, PRESENT_POSITION, raw) || !readZItem(z, MOVING, moving) ||
          !expectZItem(z, TORQUE_ENABLE, 1) || !expectZItem(z, HARDWARE_ERROR_STATUS, 0)) return false;
      if (raw < zCalibration[z].minRaw || raw > zCalibration[z].maxRaw)
        return zFailure(z, F("position_outside_limits"));
      if (labs(raw - zNeutralRaw[z]) > motors[Z_INDICES[z]].tolerance || moving != 0) {
        settled = false;
        faultMotorIndex = Z_INDICES[z];
      }
    }
    unsigned long observedMs = millis();
    if (observedMs - startMs >= Z_SETTLE_TIMEOUT_MS) break;
    if (zSettleObservation(settled, observedMs, startMs, observingStable, stableSinceMs)) {
      zAxisReady = true;
      if (!revalidateZReady()) return false;
      if (millis() - startMs >= Z_SETTLE_TIMEOUT_MS) break;
      faultMotorIndex = -1;
      CMD_SERIAL.print(F("OK,z_axis_holding_raw="));
      for (int z = 0; z < Z_MOTOR_COUNT; z++) {
        CMD_SERIAL.print(motors[Z_INDICES[z]].id);
        CMD_SERIAL.print(':');
        CMD_SERIAL.print(zNeutralRaw[z]);
        if (z + 1 < Z_MOTOR_COUNT) CMD_SERIAL.print('/');
      }
      CMD_SERIAL.println();
      return true;
    }
    delay(10);
  }
  stopForFault("z_settle_timeout");
  return false;
}

bool validateLegStartup() {
  float q[2];
  if (!inverseKinematics(START_X, START_Y, q)) return false;
  for (int i = 0; i < LEG_MOTOR_COUNT; i++) {
    const LegStartupCalibration &c = legStartup[i];
    legStartRaw[i] = jointRadToMotorRaw(i, q[i % 2]);
    if (!c.verified || c.minRaw < 0 || c.maxRaw > 4095 || c.minRaw >= c.maxRaw ||
        c.startToleranceRaw < 0 || c.startToleranceRaw > c.maxRaw - c.minRaw ||
        c.holdToleranceRaw < 0 || c.holdToleranceRaw > c.startToleranceRaw ||
        legStartRaw[i] < c.minRaw || legStartRaw[i] > c.maxRaw ||
        (c.expectedDriveMode != 0 && c.expectedDriveMode != 1) ||
        c.expectedHomingOffsetRaw < -1024 || c.expectedHomingOffsetRaw > 1024) return false;
  }
  return true;
}

bool legFailure(int i) {
  faultMotorIndex = i;
  stopForFault("walking_motor_check_failed");
  return false;
}

bool readLegStartupPosition(int i, int32_t &raw) {
  const LegStartupCalibration &c = legStartup[i];
  int32_t velocity;
  return readMotorItem(i, PRESENT_POSITION, raw) &&
         readMotorItem(i, PRESENT_VELOCITY, velocity) && velocity == 0 &&
         raw >= c.minRaw && raw <= c.maxRaw &&
         raw - c.expectedHomingOffsetRaw >= 0 && raw - c.expectedHomingOffsetRaw <= 4095 &&
         labs(raw - legStartRaw[i]) <= c.startToleranceRaw;
}

bool prepareLegMotor(int i) {
  const LegStartupCalibration &c = legStartup[i];
  int32_t secondary, firmware;
  // Provision EEPROM separately. Rewriting operating mode here can reset gains.
  if (!expectMotorItem(i, MODEL_NUMBER, XM430_W350) ||
      !expectMotorItem(i, TORQUE_ENABLE, 0) || !expectMotorItem(i, OPERATING_MODE, 3) ||
      !expectMotorItem(i, DRIVE_MODE, c.expectedDriveMode) ||
      !expectMotorItem(i, HOMING_OFFSET, c.expectedHomingOffsetRaw) ||
      !expectMotorItem(i, MIN_POSITION_LIMIT, c.minRaw) ||
      !expectMotorItem(i, MAX_POSITION_LIMIT, c.maxRaw) ||
      !expectMotorItem(i, STATUS_RETURN_LEVEL, 2) ||
      !expectMotorItem(i, REGISTERED_INSTRUCTION, 0) ||
      !expectMotorItem(i, HARDWARE_ERROR_STATUS, 0) || !expectMotorItem(i, BUS_WATCHDOG, 0) ||
      !readMotorItem(i, SECONDARY_ID, secondary) || secondary < 253 ||
      !readMotorItem(i, FIRMWARE_VERSION, firmware) || firmware <= 0 ||
      (firmware >= 45 && !expectMotorItem(i, STARTUP_CONFIGURATION, 0)) ||
      // Preserve the original leg profile. Velocity zero is unlimited, NOT slow.
      !writeMotorItem(i, PROFILE_VELOCITY, 0) || !expectMotorItem(i, PROFILE_VELOCITY, 0) ||
      !writeMotorItem(i, PROFILE_ACCELERATION, 20) || !expectMotorItem(i, PROFILE_ACCELERATION, 20) ||
      !readLegStartupPosition(i, legHoldRaw[i])) return legFailure(i);
  return true;
}

bool setupOneMotor(int i) {
  if (!zCalibrationReady || !legStartupReady || !MOTION_COMMISSIONING_VERIFIED ||
      !zAxisReady || mode == MODE_STOPPED || i < 0 || i >= LEG_MOTOR_COUNT) return false;
  if (!revalidateZReady()) return false;
  int32_t enabledRaw;
  // A measured absolute goal is verified while torque is still off.
  if (!readLegStartupPosition(i, legHoldRaw[i]) ||
      !writeMotorItem(i, GOAL_POSITION, legHoldRaw[i]) ||
      !expectMotorItem(i, GOAL_POSITION, legHoldRaw[i]) || !expectMotorItem(i, TORQUE_ENABLE, 0))
    return legFailure(i);
  torqueOffUnconfirmed = true;
  if (!writeMotorItem(i, TORQUE_ENABLE, 1) || !expectMotorItem(i, TORQUE_ENABLE, 1) ||
      !readLegStartupPosition(i, enabledRaw) ||
      labs(enabledRaw - legHoldRaw[i]) > legStartup[i].holdToleranceRaw ||
      !expectMotorItem(i, HARDWARE_ERROR_STATUS, 0)) return legFailure(i);
  return true;
}

bool verifyLegStartHolds() {
  if (!legStartupReady || !MOTION_COMMISSIONING_VERIFIED) return false;
  for (int i = 0; i < LEG_MOTOR_COUNT; i++) {
    int32_t raw;
    if (!readLegStartupPosition(i, raw) ||
        labs(raw - legHoldRaw[i]) > legStartup[i].holdToleranceRaw ||
        !expectMotorItem(i, GOAL_POSITION, legHoldRaw[i]) ||
        !expectMotorItem(i, TORQUE_ENABLE, 1) || !expectMotorItem(i, HARDWARE_ERROR_STATUS, 0))
      return legFailure(i);
  }
  return true;
}

bool setupAllMotors() {
  zCalibrationReady = validateZCalibration();
  legStartupReady = validateLegStartup();
  if (!zCalibrationReady || !legStartupReady || !MOTION_COMMISSIONING_VERIFIED || mode == MODE_STOPPED) return false;
  // Disable all configured motors before Z preparation. A failure blocks startup.
  safeCleanup();
  if (torqueOffUnconfirmed) return false;
  for (int i = 0; i < LEG_MOTOR_COUNT; i++) {
    pollCommandSerial();
    if (mode == MODE_STOPPED || !prepareLegMotor(i)) return false;
  }
  if (!holdZAxis()) return false;
  for (int i = 0; i < LEG_MOTOR_COUNT; i++) {
    pollCommandSerial();
    if (mode == MODE_STOPPED) return false;
    if (!setupOneMotor(i)) {
      return false;
    }
  }
  torqueEnabled = true;
  return true;
}

void commandMotorModelRad(int motorIndex, float modelRad) {
  if (!zCalibrationReady || !legStartupReady || !MOTION_COMMISSIONING_VERIFIED ||
      !zAxisReady || !hardwareReady || mode == MODE_STOPPED ||
      motorIndex < 0 || motorIndex >= LEG_MOTOR_COUNT) return;
  if (!zFeedbackFresh()) { stopForFault("z_feedback_stale"); return; }
  if (isInvalid(modelRad)) { legFailure(motorIndex); return; }

  int32_t raw = jointRadToMotorRaw(motorIndex, modelRad);

  if (raw < legStartup[motorIndex].minRaw || raw > legStartup[motorIndex].maxRaw ||
      !writeMotorItem(motorIndex, GOAL_POSITION, raw)) legFailure(motorIndex);
}

void writeAllLegsRad() {
  // Order mirrors MATLAB writeAllLegsSync order:
  // [FR thigh, FR crank, FL thigh, FL crank, RR thigh, RR crank, RL thigh, RL crank]
  commandMotorModelRad(FR_THIGH, previousAction[LEG_FR][0]);
  commandMotorModelRad(FR_CRANK, previousAction[LEG_FR][1]);

  commandMotorModelRad(FL_THIGH, previousAction[LEG_FL][0]);
  commandMotorModelRad(FL_CRANK, previousAction[LEG_FL][1]);

  commandMotorModelRad(RR_THIGH, previousAction[LEG_RR][0]);
  commandMotorModelRad(RR_CRANK, previousAction[LEG_RR][1]);

  commandMotorModelRad(RL_THIGH, previousAction[LEG_RL][0]);
  commandMotorModelRad(RL_CRANK, previousAction[LEG_RL][1]);
}

void safeCleanup() {
  hardwareReady = false;
  zAxisReady = false;
  torqueEnabled = false;
  bool allOff = true;
  torqueOffDispatchAcknowledged = true;
  int order[ALL_MOTOR_COUNT];
  int count = 0;
  if (faultMotorIndex >= 0 && faultMotorIndex < ALL_MOTOR_COUNT) order[count++] = faultMotorIndex;
  for (int z = 0; z < Z_MOTOR_COUNT; z++)
    if (Z_INDICES[z] != faultMotorIndex) order[count++] = Z_INDICES[z];
  for (int i = 0; i < LEG_MOTOR_COUNT; i++)
    if (i != faultMotorIndex) order[count++] = i;
  // Dispatch every disable before readback. Each request waits at most the short
  // receive timeout; consuming replies avoids half-duplex RS-485 collisions.
  for (int n = 0; n < count; n++) {
    int i = order[n];
    if (!configuredMotorId(motors[i].id)) {
      allOff = false;
      torqueOffDispatchAcknowledged = false;
      continue;
    }
    selectMotorProtocol(i);
    uint8_t off = 0;
    if (!dxl.write(motors[i].id, 64, &off, 1, DXL_IO_TIMEOUT_MS) || !dxlExchangeOK())
      torqueOffDispatchAcknowledged = false;
  }
  for (int n = 0; n < count; n++) {
    int i = order[n];
    if (!configuredMotorId(motors[i].id) || !expectMotorItem(i, TORQUE_ENABLE, 0)) allOff = false;
  }
  torqueOffUnconfirmed = !allOff;
  CMD_SERIAL.print(F("STATE,torque_off_dispatch_ack="));
  CMD_SERIAL.println(torqueOffDispatchAcknowledged ? 1 : 0);
  if (allOff) sendLine("OK,cleanup_done");
  else CMD_SERIAL.println(F("WARN,cleanup_incomplete_unknown_or_unresponsive_motors"));
}

void stopForFault(const char* reason) {
  mode = MODE_STOPPED;
  safeCleanup();
  sendModeState(reason);
}

bool readZFeedback(int z) {
  // One contiguous Protocol 2.0 read: Torque Enable(64)..Present Position(135).
  // Validate this 72-byte read and the latency on the actual firmware/RS-485 bus.
  uint8_t data[72];
  int i = Z_INDICES[z];
  unsigned long sampledMs = millis();
  unsigned long startedUs = micros();
  selectMotorProtocol(i);
  if (dxl.read(motors[i].id, 64, sizeof(data), data, sizeof(data), DXL_IO_TIMEOUT_MS) != sizeof(data) ||
      !dxlExchangeOK()) return zFailure(z, F("feedback_read_failed"));
  if (micros() - startedUs > Z_MONITOR_BUDGET_US) return zFailure(z, F("feedback_budget_exceeded"));
  uint32_t bits = (uint32_t)data[68] | ((uint32_t)data[69] << 8) |
                  ((uint32_t)data[70] << 16) | ((uint32_t)data[71] << 24);
  int32_t raw;
  memcpy(&raw, &bits, sizeof(raw));
  if (data[0] != 1 || data[6] != 0 || data[58] != 0 ||
      raw < zCalibration[z].minRaw || raw > zCalibration[z].maxRaw ||
      labs(raw - zNeutralRaw[z]) > motors[i].tolerance)
    return zFailure(z, F("holding_readiness_lost"));
  zLastGoodMs[z] = sampledMs;
  return true;
}

bool revalidateZReady() {
  if (!zCalibrationReady || !zAxisReady || !MOTION_COMMISSIONING_VERIFIED || mode == MODE_STOPPED) return false;
  for (int z = 0; z < Z_MOTOR_COUNT; z++) if (!readZFeedback(z)) return false;
  nextZMonitorMs = millis() + Z_MONITOR_PERIOD_MS;
  return true;
}

bool zFeedbackFresh() {
  if (!zAxisReady) return false;
  unsigned long now = millis();
  for (int z = 0; z < Z_MOTOR_COUNT; z++) {
    if (now - zLastGoodMs[z] > Z_MONITOR_MAX_AGE_MS) {
      faultMotorIndex = Z_INDICES[z];
      return false;
    }
  }
  return true;
}

void serviceZMonitor(unsigned long deadlineMs) {
  if (!hardwareReady || mode == MODE_STOPPED) return;
  if (!zFeedbackFresh()) { stopForFault("z_feedback_stale"); return; }
  unsigned long now = millis();
  if ((long)(now - nextZMonitorMs) < 0 || (long)(deadlineMs - now) < 4) return;
  if (!readZFeedback(nextZMonitor)) return;
  nextZMonitor = (nextZMonitor + 1) % Z_MOTOR_COUNT;
  nextZMonitorMs = millis() + Z_MONITOR_PERIOD_MS;
}

// =====================================================
// Kinematics translated from MATLAB
// =====================================================
bool inverseKinematics(float x, float y, float q[2]) {
  float DG = L2_DG;
  float GH = L3_GH;

  float d_DH_sq = x*x + y*y;
  float d_DH = sqrtf(d_DH_sq);

  if (d_DH > (DG + GH) || d_DH < fabsf(DG - GH) || d_DH < 0.0001f) {
    return false;
  }

  float alpha = atan2f(y, x);
  float cos_delta = (DG*DG + d_DH_sq - GH*GH) / (2.0f * DG * d_DH);
  if (cos_delta < -1.0f || cos_delta > 1.0f) return false;
  float delta = acosf(clampFloat(cos_delta, -1.0f, 1.0f));

  float thigh_angle = alpha + ((float)BRANCH_KNEE * delta);
  float theta2 = thigh_angle - PI_F/2.0f;

  float G_x = DG * cosf(thigh_angle);
  float G_y = DG * sinf(thigh_angle);

  float shin_angle = atan2f(y - G_y, x - G_x);
  float F_x = G_x + FG * cosf(shin_angle + PI_F);
  float F_y = G_y + FG * sinf(shin_angle + PI_F);

  float d_DF_sq = F_x*F_x + F_y*F_y;
  float d_DF = sqrtf(d_DF_sq);
  if (d_DF < 0.0001f) return false;

  float phi_DF = atan2f(F_y, F_x);
  float cos_gamma = (DE*DE + d_DF_sq - EF*EF) / (2.0f * DE * d_DF);
  if (cos_gamma < -1.0f || cos_gamma > 1.0f) return false;
  float gamma = acosf(clampFloat(cos_gamma, -1.0f, 1.0f));

  float phi_DE = phi_DF + ((float)BRANCH_E * gamma);

  float cos_CDE = (CD*CD + DE*DE - CE*CE) / (2.0f * CD * DE);
  float ang_CDE = acosf(clampFloat(cos_CDE, -1.0f, 1.0f));

  float phi_CD = phi_DE - ang_CDE;
  float C_x = CD * cosf(phi_CD);
  float C_y = CD * sinf(phi_CD);

  float AC_x = C_x - AD;
  float AC_y = C_y;
  float d_AC_sq = AC_x*AC_x + AC_y*AC_y;
  float d_AC = sqrtf(d_AC_sq);
  if (d_AC < 0.0001f) return false;

  float phi_AC = atan2f(AC_y, AC_x);
  float cos_lambda = (AB*AB + d_AC_sq - BC*BC) / (2.0f * AB * d_AC);
  if (cos_lambda < -1.0f || cos_lambda > 1.0f) return false;
  float lambda = acosf(clampFloat(cos_lambda, -1.0f, 1.0f));

  float a = phi_AC + ((float)BRANCH_CRANK * lambda) - A_OFFSET_RAD;

  if (isInvalid(theta2) || isInvalid(a)) return false;

  q[0] = theta2;
  q[1] = a;
  return true;
}

bool forwardKinematics(float theta2, float a, float &x, float &y) {
  float DG = L2_DG;
  float GH = L3_GH;

  float a_geo = a + A_OFFSET_RAD;
  float B_x = AD + AB * cosf(a_geo);
  float B_y = AB * sinf(a_geo);

  float d_DB_sq = B_x*B_x + B_y*B_y;
  float d_DB = sqrtf(d_DB_sq);
  if (d_DB < 0.0001f) return false;

  float phi_DB = atan2f(B_y, B_x);
  float cos_CDB = (CD*CD + d_DB_sq - BC*BC) / (2.0f * CD * d_DB);
  if (cos_CDB < -1.0f || cos_CDB > 1.0f) return false;
  float ang_CDB = acosf(clampFloat(cos_CDB, -1.0f, 1.0f));

  float phi_CD = phi_DB + ((float)BRANCH_CRANK * ang_CDB);

  float cos_CDE = (CD*CD + DE*DE - CE*CE) / (2.0f * CD * DE);
  float ang_CDE = acosf(clampFloat(cos_CDE, -1.0f, 1.0f));

  float phi_DE = phi_CD + ang_CDE;
  float E_x = DE * cosf(phi_DE);
  float E_y = DE * sinf(phi_DE);

  float thigh_angle = theta2 + PI_F/2.0f;
  float G_x = DG * cosf(thigh_angle);
  float G_y = DG * sinf(thigh_angle);

  float d_EG = hypotf(E_x - G_x, E_y - G_y);
  if (d_EG > (EF + FG) || d_EG < fabsf(EF - FG) || d_EG < 0.0001f) {
    return false;
  }

  float a_len = (EF*EF - FG*FG + d_EG*d_EG) / (2.0f * d_EG);
  float h_sq = EF*EF - a_len*a_len;
  if (h_sq < 0.0f) h_sq = 0.0f;
  float h_off = sqrtf(h_sq);

  float ux = (G_x - E_x) / d_EG;
  float uy = (G_y - E_y) / d_EG;
  float Mx = E_x + a_len * ux;
  float My = E_y + a_len * uy;

  float F_x = Mx - ((float)BRANCH_E * h_off * uy);
  float F_y = My + ((float)BRANCH_E * h_off * ux);

  float shin_dx = G_x - F_x;
  float shin_dy = G_y - F_y;
  float nrm = hypotf(shin_dx, shin_dy);
  if (nrm < 0.0001f) return false;

  x = G_x + GH * shin_dx / nrm;
  y = G_y + GH * shin_dy / nrm;

  return !(isInvalid(x) || isInvalid(y));
}

// =====================================================
// Trajectory and controller functions
// =====================================================
void getCycleReferenceByIndex(int idx, float &refX, float &refY) {
  idx = idx % N_CYC;
  if (idx < 0) idx += N_CYC;

  float T_cycle = 1.0f / FREQ;
  float t = ((float)idx) * TS;

  if (t < (T_cycle / 2.0f)) {
    float tau = t / (T_cycle / 2.0f);
    refX = (XC - AMP_X) + (2.0f * AMP_X / TWO_PI_F) * (TWO_PI_F * tau - sinf(TWO_PI_F * tau));
    refY = (YC - AMP_Y) + AMP_Y * (1.0f - cosf(TWO_PI_F * tau));
  }
  else {
    float tau = (t - (T_cycle / 2.0f)) / (T_cycle / 2.0f);
    refX = (XC + AMP_X) - (2.0f * AMP_X * tau);
    refY = (YC - AMP_Y);
  }
}

void computeAction(float refX, float refY, float prevAction[2]) {
  float target[2];
  bool ok = inverseKinematics(refX, refY, target);

  if (!ok) {
    // Same safety idea as MATLAB: if IK fails, hold previous action.
    return;
  }

  for (int i = 0; i < 2; i++) {
    float delta = target[i] - prevAction[i];
    delta = clampFloat(delta, -MAX_DELTA_ANGLE, MAX_DELTA_ANGLE);
    prevAction[i] = prevAction[i] + delta;
  }
}

bool verifyAllReferences() {
  float qTest[2];

  if (!inverseKinematics(STAND_X, STAND_Y, qTest)) {
    sendLine("ERR,standing_point_unreachable");
    return false;
  }

  if (!inverseKinematics(START_X, START_Y, qTest)) {
    sendLine("ERR,start_point_unreachable");
    return false;
  }

  for (int k = 0; k < N_CYC; k++) {
    float x, y;
    getCycleReferenceByIndex(k, x, y);

    if (!inverseKinematics(x, y, qTest)) {
      CMD_SERIAL.print("ERR,trajectory_unreachable,k=");
      CMD_SERIAL.print(k);
      CMD_SERIAL.print(",x=");
      CMD_SERIAL.print(x, 2);
      CMD_SERIAL.print(",y=");
      CMD_SERIAL.println(y, 2);
      return false;
    }
  }

  sendLine("OK,all_references_reachable");
  return true;
}

void setAllLegActions(const float q[2]) {
  for (int leg = 0; leg < 4; leg++) {
    previousAction[leg][0] = q[0];
    previousAction[leg][1] = q[1];
    lastRef[leg][0] = STAND_X;
    lastRef[leg][1] = STAND_Y;
  }
}

bool smoothStandupBlocking() {
  if (!zCalibrationReady || !zAxisReady || !hardwareReady || mode == MODE_STOPPED ||
      !legStartupReady || !MOTION_COMMISSIONING_VERIFIED) return false;
  float qStart[2];
  float qStand[2];

  if (!inverseKinematics(START_X, START_Y, qStart)) {
    sendLine("ERR,standup_start_ik_failed");
    return false;
  }

  if (!inverseKinematics(STAND_X, STAND_Y, qStand)) {
    sendLine("ERR,standup_stand_ik_failed");
    return false;
  }

  CMD_SERIAL.print("INFO,smooth_standup_seconds=");
  CMD_SERIAL.println(STANDUP_DURATION, 1);

  // Check measured holds and Z feedback immediately before the first ramp goal.
  if (!verifyLegStartHolds() || !revalidateZReady()) return false;

  for (int step = 1; step <= STANDUP_STEPS; step++) {
    pollCommandSerial();
    if (mode == MODE_STOPPED) return false;
    unsigned long stepStart = millis();

    float alpha = (float)step / (float)STANDUP_STEPS;
    float alphaSmooth = alpha * alpha * (3.0f - 2.0f * alpha);

    float qRamp[2];
    qRamp[0] = qStart[0] + alphaSmooth * (qStand[0] - qStart[0]);
    qRamp[1] = qStart[1] + alphaSmooth * (qStand[1] - qStart[1]);

    setAllLegActions(qRamp);
    writeAllLegsRad();
    if (mode == MODE_STOPPED) return false;

    // Keep accepting emergency stop/status requests during standup.
    while (millis() - stepStart < TS_MS) {
      pollCommandSerial();
      if (mode == MODE_STOPPED) return false;
      serviceZMonitor(stepStart + TS_MS);
      if (mode == MODE_STOPPED) return false;
      delay(1);
    }
  }

  setAllLegActions(qStand);
  writeAllLegsRad();
  if (mode == MODE_STOPPED) return false;
  sendLine("OK,standup_complete");
  return true;
}

// =====================================================
// UART2 command parser: Raspberry Pi -> Arduino
// =====================================================
void uppercaseInPlace(char* s) {
  for (uint8_t i = 0; s[i] != '\0'; i++) {
    s[i] = (char)toupper((unsigned char)s[i]);
  }
}

void setMode(RunMode newMode, const char* reason) {
  if (mode == MODE_STOPPED && newMode != MODE_STOPPED) {
    sendLine("ERR,motors_stopped_reset_arduino_to_reenable");
    return;
  }

  if (newMode != MODE_STOPPED && (!zCalibrationReady || !legStartupReady ||
      !MOTION_COMMISSIONING_VERIFIED || !zAxisReady || !hardwareReady)) {
    CMD_SERIAL.println(F("ERR,z_axis_not_ready_motion_blocked"));
    return;
  }

  if (newMode != MODE_STOPPED && !zFeedbackFresh()) {
    stopForFault("z_feedback_stale");
    return;
  }

  mode = newMode;

  if (mode == MODE_STANDING) {
    phaseIdx = 0;
  }

  sendModeState(reason);
}

void toggleWalkStand() {
  if (mode == MODE_WALKING) {
    setMode(MODE_STANDING, "toggle");
  } else if (mode == MODE_STANDING) {
    setMode(MODE_WALKING, "toggle");
  } else {
    sendLine("ERR,cannot_toggle_when_stopped");
  }
}

void handleCommand(char* cmd) {
  uppercaseInPlace(cmd);

  if (strcmp(cmd, "W") == 0 || strcmp(cmd, "WALK") == 0 || strcmp(cmd, "START") == 0) {
    setMode(MODE_WALKING, "uart2_command");
  }
  else if (strcmp(cmd, "S") == 0 || strcmp(cmd, "STAND") == 0 || strcmp(cmd, "STANDING") == 0) {
    setMode(MODE_STANDING, "uart2_command");
  }
  else if (strcmp(cmd, "T") == 0 || strcmp(cmd, "TOGGLE") == 0 || strcmp(cmd, "SPACE") == 0) {
    toggleWalkStand();
  }
  else if (strcmp(cmd, "Q") == 0 || strcmp(cmd, "QUIT") == 0 || strcmp(cmd, "DISABLE") == 0 || strcmp(cmd, "STOP") == 0) {
    mode = MODE_STOPPED;
    safeCleanup();
    sendModeState("uart2_quit");
  }
  else if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "STATE?") == 0 || strcmp(cmd, "PING") == 0 || strcmp(cmd, "SCAN") == 0) {
    sendStatus(true);
  }
  else if (strcmp(cmd, "HELP") == 0) {
    sendLine("CMDS,WALK/W/START,STAND/S,TOGGLE/T/SPACE,QUIT/Q/STOP/DISABLE,STATUS/STATE?/PING/SCAN,HELP");
  }
  else if (cmd[0] != '\0') {
    CMD_SERIAL.print("ERR,unknown_cmd=");
    CMD_SERIAL.println(cmd);
  }
}

void pollCommandSerial() {
  while (CMD_SERIAL.available() > 0) {
    char c = (char)CMD_SERIAL.read();

    if (c == ' ' && cmdLen == 0) {
      char tmp[] = "TOGGLE";
      handleCommand(tmp);
      continue;
    }

    if (c == '\r' || c == '\n') {
      if (cmdLen > 0) {
        cmdBuffer[cmdLen] = '\0';
        handleCommand(cmdBuffer);
        cmdLen = 0;
      }
      continue;
    }

    if (c == '\t') {
      continue;
    }

    if (cmdLen < sizeof(cmdBuffer) - 1) {
      cmdBuffer[cmdLen++] = c;
    } else {
      cmdLen = 0;
      sendLine("ERR,command_too_long");
    }
  }
}

// ============================
// Implémentation gyro
// ============================

// Envoi de la trame IMU vers la Raspberry Pi via UART2
void sendImuData() {
  if (!bnoReady) return;

  // Lecture des vecteurs (accélération en m/s², gyro en rad/s)
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro  = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  // Format : IMU,ax,ay,az,gx,gy,gz
  CMD_SERIAL.print("IMU,");
  CMD_SERIAL.print(accel.x(), 3); CMD_SERIAL.print(",");
  CMD_SERIAL.print(accel.y(), 3); CMD_SERIAL.print(",");
  CMD_SERIAL.print(accel.z(), 3); CMD_SERIAL.print(",");
  CMD_SERIAL.print(gyro.x(), 3);  CMD_SERIAL.print(",");
  CMD_SERIAL.print(gyro.y(), 3);  CMD_SERIAL.print(",");
  CMD_SERIAL.println(gyro.z(), 3);
}

// =====================================================
// Arduino setup and nonblocking control loop
// =====================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  CMD_SERIAL.begin(CMD_BAUDRATE);
  delay(300);
  sendLine("BOOT,uart2_ready,baud=115200,robot=quadruped_12xm430");

//=========================
// Implémentation gyro
//=========================
  // --- AJOUT : Initialisation BNO055 ---
  Wire.begin();
  Wire.setClock(400000); // I2C Fast Mode (400 kHz) pour minimiser le temps de lecture

  if (bno.begin()) {
    bno.setExtCrystalUse(true);
    bnoReady = true;
    sendLine("OK,bno055_initialized");
  } else {
    sendLine("WARN,bno055_not_found_check_wiring");
  }
  // ------------------------------------

  sendLine("INFO,connect_raspberrypi_TX_to_Arduino_RX2_pin17");
  sendLine("INFO,connect_raspberrypi_RX_to_Arduino_TX2_pin16_level_shift_recommended");

  dxl.begin(DXL_BAUDRATE);
  delay(500);

  if (!verifyAllReferences()) {
    mode = MODE_STOPPED;
    sendModeState("workspace_check_failed");
    return;
  }

  if (!setupAllMotors()) {
    mode = MODE_STOPPED;
    safeCleanup();
    sendModeState("motor_setup_failed");
    return;
  }

  hardwareReady = true;

  sendLine("INFO,moving_to_standing_position_smoothly");
  if (!smoothStandupBlocking()) {
    mode = MODE_STOPPED;
    safeCleanup();
    sendModeState("standup_failed");
    return;
  }

  phaseIdx = 0;
  nextLoopMs = millis();
  lastStatusMs = millis();

  sendModeState("ready");
  sendLine("CMDS,WALK/W/START,STAND/S,TOGGLE/T/SPACE,QUIT/Q/STOP/DISABLE,STATUS/STATE?/PING/SCAN,HELP");
}

void loop() {
  pollCommandSerial();

  if (mode == MODE_STOPPED || !hardwareReady || !zCalibrationReady || !zAxisReady) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(20);
    return;
  }

  if (!zFeedbackFresh()) { stopForFault("z_feedback_stale"); return; }

  unsigned long nowMs = millis();

  if ((long)(nowMs - nextLoopMs) >= 0) {
    nextLoopMs += TS_MS;

    if (mode == MODE_WALKING) {
      // MATLAB behavior: phase advances only while walking.
      phaseIdx = (phaseIdx + 1) % N_CYC;

      int idxFR = (phaseIdx + SHIFT_FR) % N_CYC;
      int idxFL = (phaseIdx + SHIFT_FL) % N_CYC;
      int idxRR = (phaseIdx + SHIFT_RR) % N_CYC;
      int idxRL = (phaseIdx + SHIFT_RL) % N_CYC;

      getCycleReferenceByIndex(idxFR, lastRef[LEG_FR][0], lastRef[LEG_FR][1]);
      getCycleReferenceByIndex(idxFL, lastRef[LEG_FL][0], lastRef[LEG_FL][1]);
      getCycleReferenceByIndex(idxRR, lastRef[LEG_RR][0], lastRef[LEG_RR][1]);
      getCycleReferenceByIndex(idxRL, lastRef[LEG_RL][0], lastRef[LEG_RL][1]);
    }
    else {
      // STANDING: hold all feet at standing point and reset gait phase.
      phaseIdx = 0;
      for (int leg = 0; leg < 4; leg++) {
        lastRef[leg][0] = STAND_X;
        lastRef[leg][1] = STAND_Y;
      }
    }

    for (int leg = 0; leg < 4; leg++) {
      computeAction(lastRef[leg][0], lastRef[leg][1], previousAction[leg]);
    }

    writeAllLegsRad();
    if (mode == MODE_STOPPED) return;

    if (mode == MODE_WALKING) {
      digitalWrite(LED_BUILTIN, (phaseIdx % 10) < 5 ? HIGH : LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  serviceZMonitor(nextLoopMs);
  if (mode == MODE_STOPPED) return;

  if (nowMs - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    sendStatus(false);  // periodic scan without motor reads, to keep 100 Hz loop light.
  }
  // --- AJOUT : Envoi périodique IMU (50 Hz) ---
  if (nowMs - lastImuMs >= IMU_PERIOD_MS) {
    lastImuMs = nowMs;
    sendImuData();
  }
  // ---------------------------------------------
}
