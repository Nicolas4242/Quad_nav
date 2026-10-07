#include <Dynamixel2Arduino.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

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
const unsigned long STATUS_PERIOD_MS = 500;

// =====================================================
// Motor types and configuration
// =====================================================
enum MotorType {
  MOTOR_XM430_W350_R,
  MOTOR_AX12A
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
// AX order matches MATLAB hw_params.AX_IDS: [11,12,13,14] = [FL-Z, FR-Z, RL-Z, RR-Z]
enum MotorIndex {
  FR_THIGH = 0,
  FR_CRANK = 1,
  FL_THIGH = 2,
  FL_CRANK = 3,
  RR_THIGH = 4,
  RR_CRANK = 5,
  RL_THIGH = 6,
  RL_CRANK = 7,
  AX_FL_Z  = 8,
  AX_FR_Z  = 9,
  AX_RL_Z  = 10,
  AX_RR_Z  = 11
};

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

  {"AX_FL_Z",  11,  MOTOR_AX12A,        1.0f, 300.0f, 1023, 10,  1,   0.0f},
  {"AX_FR_Z",  12,  MOTOR_AX12A,        1.0f, 300.0f, 1023, 10,  1,   0.0f},
  {"AX_RL_Z",  13,  MOTOR_AX12A,        1.0f, 300.0f, 1023, 10,  1,   0.0f},
  {"AX_RR_Z",  14,  MOTOR_AX12A,        1.0f, 300.0f, 1023, 10,  1,   0.0f}
};

const int LEG_MOTOR_COUNT = 8;
const int AX_MOTOR_COUNT = 4;
const int ALL_MOTOR_COUNT = sizeof(motors) / sizeof(motors[0]);

const float AX_HOLD_DEG[AX_MOTOR_COUNT] = {152.0f, 152.0f, 150.0f, 148.0f};
const int AX_INDICES[AX_MOTOR_COUNT] = {AX_FL_Z, AX_FR_Z, AX_RL_Z, AX_RR_Z};

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

char cmdBuffer[64];
uint8_t cmdLen = 0;

// Forward declaration because smoothStandupBlocking() keeps UART2 active.
void pollCommandSerial();

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
      dxl.setPortProtocolVersion(motors[i].protocol);
      int32_t raw = (int32_t)dxl.getPresentPosition(motors[i].id, UNIT_RAW);
      CMD_SERIAL.print(motors[i].id);
      CMD_SERIAL.print(":");
      CMD_SERIAL.print(raw);
      if (i < LEG_MOTOR_COUNT - 1) CMD_SERIAL.print("/");
    }

    CMD_SERIAL.print(",raw_z=");
    for (int j = 0; j < AX_MOTOR_COUNT; j++) {
      int idx = AX_INDICES[j];
      dxl.setPortProtocolVersion(motors[idx].protocol);
      int32_t raw = (int32_t)dxl.getPresentPosition(motors[idx].id, UNIT_RAW);
      CMD_SERIAL.print(motors[idx].id);
      CMD_SERIAL.print(":");
      CMD_SERIAL.print(raw);
      if (j < AX_MOTOR_COUNT - 1) CMD_SERIAL.print("/");
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
  MotorConfig m = motors[motorIndex];

  if (m.type == MOTOR_XM430_W350_R) {
    motorAbsRad = normalizeRadPositive(motorAbsRad);
    int32_t raw = (int32_t)roundf(motorAbsRad * (4096.0f / TWO_PI_F));
    raw = (int32_t)clampFloat((float)raw, 0.0f, 4095.0f);
    return raw;
  }

  // AX-12A: 0..300 deg -> 0..1023.
  float maxRad = m.maxDegree * DEG_TO_RAD_F;
  motorAbsRad = clampFloat(motorAbsRad, 0.0f, maxRad);
  int32_t raw = (int32_t)roundf(motorAbsRad * ((float)m.maxRaw / maxRad));
  raw = (int32_t)clampFloat((float)raw, 0.0f, (float)m.maxRaw);
  return raw;
}

float motorRawToAbsRad(int motorIndex, int32_t raw) {
  MotorConfig m = motors[motorIndex];

  if (m.type == MOTOR_XM430_W350_R) {
    raw = (int32_t)clampFloat((float)raw, 0.0f, 4095.0f);
    return ((float)raw) * (TWO_PI_F / 4096.0f);
  }

  raw = (int32_t)clampFloat((float)raw, 0.0f, (float)m.maxRaw);
  float maxRad = m.maxDegree * DEG_TO_RAD_F;
  return ((float)raw) * (maxRad / (float)m.maxRaw);
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
bool setupOneMotor(int motorIndex) {
  MotorConfig m = motors[motorIndex];

  selectMotorProtocol(motorIndex);
  delay(20);

  if (!dxl.ping(m.id)) {
    CMD_SERIAL.print("ERR,ping_failed,id=");
    CMD_SERIAL.println(m.id);
    return false;
  }

  dxl.torqueOff(m.id);
  delay(80);

  if (m.type == MOTOR_XM430_W350_R) {
    dxl.setOperatingMode(m.id, OP_POSITION);
    delay(50);

    // MATLAB DynamixelInterface: safe_speed_val = 0 for X-Series profile velocity.
    dxl.writeControlTableItem(PROFILE_VELOCITY, m.id, 0);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, m.id, 20);
  }
  else if (m.type == MOTOR_AX12A) {
    // AX-12 joint mode, 0..300 degrees.
    dxl.writeControlTableItem(CW_ANGLE_LIMIT, m.id, 0);
    dxl.writeControlTableItem(CCW_ANGLE_LIMIT, m.id, m.maxRaw);

    // Equivalent intent of MATLAB: tight hold, max torque, firm speed.
    dxl.writeControlTableItem(MOVING_SPEED, m.id, 1023);
    dxl.writeControlTableItem(TORQUE_LIMIT, m.id, 1023);
  }

  delay(80);
  dxl.torqueOn(m.id);
  delay(80);

  CMD_SERIAL.print("OK,motor_ready,id=");
  CMD_SERIAL.print(m.id);
  CMD_SERIAL.print(",name=");
  CMD_SERIAL.println(m.name);

  return true;
}

bool setupAllMotors() {
  for (int i = 0; i < ALL_MOTOR_COUNT; i++) {
    if (!setupOneMotor(i)) {
      return false;
    }
  }
  torqueEnabled = true;
  return true;
}

void commandMotorModelRad(int motorIndex, float modelRad) {
  if (isInvalid(modelRad)) return;

  int32_t raw = jointRadToMotorRaw(motorIndex, modelRad);

  selectMotorProtocol(motorIndex);
  dxl.setGoalPosition(motors[motorIndex].id, raw, UNIT_RAW);
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

void holdZAxis() {
  for (int i = 0; i < AX_MOTOR_COUNT; i++) {
    int motorIndex = AX_INDICES[i];
    float rad = AX_HOLD_DEG[i] * DEG_TO_RAD_F;
    int32_t raw = motorAbsRadToRaw(motorIndex, rad);

    selectMotorProtocol(motorIndex);
    dxl.setGoalPosition(motors[motorIndex].id, raw, UNIT_RAW);
  }
}

void safeCleanup() {
  for (int i = 0; i < ALL_MOTOR_COUNT; i++) {
    selectMotorProtocol(i);
    dxl.torqueOff(motors[i].id);
    delay(40);
  }
  torqueEnabled = false;
  hardwareReady = false;
  sendLine("OK,cleanup_done");
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

  for (int step = 1; step <= STANDUP_STEPS; step++) {
    unsigned long stepStart = millis();

    float alpha = (float)step / (float)STANDUP_STEPS;
    float alphaSmooth = alpha * alpha * (3.0f - 2.0f * alpha);

    float qRamp[2];
    qRamp[0] = qStart[0] + alphaSmooth * (qStand[0] - qStart[0]);
    qRamp[1] = qStart[1] + alphaSmooth * (qStand[1] - qStart[1]);

    setAllLegActions(qRamp);
    writeAllLegsRad();

    // Keep accepting emergency stop/status requests during standup.
    while (millis() - stepStart < TS_MS) {
      pollCommandSerial();
      if (mode == MODE_STOPPED) return false;
      delay(1);
    }
  }

  setAllLegActions(qStand);
  writeAllLegsRad();
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

// =====================================================
// Arduino setup and nonblocking control loop
// =====================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  CMD_SERIAL.begin(CMD_BAUDRATE);
  delay(300);
  sendLine("BOOT,uart2_ready,baud=115200,robot=quadruped_standard_gait");
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

  holdZAxis();
  sendLine("OK,z_axis_holding_deg=11:152/12:152/13:150/14:148");

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

  if (mode == MODE_STOPPED || !hardwareReady) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(20);
    return;
  }

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

    if (mode == MODE_WALKING) {
      digitalWrite(LED_BUILTIN, (phaseIdx % 10) < 5 ? HIGH : LOW);
    } else {
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  if (nowMs - lastStatusMs >= STATUS_PERIOD_MS) {
    lastStatusMs = nowMs;
    sendStatus(false);  // periodic scan without motor reads, to keep 100 Hz loop light.
  }
}
