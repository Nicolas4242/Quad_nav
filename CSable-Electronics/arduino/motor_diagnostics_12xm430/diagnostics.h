#pragma once
#include <stdint.h>
#include <string.h>

namespace Diagnostics {
const uint16_t UNSET = 255;
enum OperatingMode { CURRENT_8XM, FUTURE_12XM };
struct Motor { const char* role; uint16_t id; bool replacementVerified; };
// Only physically identified replacement IDs belong here. Never infer them.
const Motor motors[] = {
  {"FR_THIGH", 1, false}, {"FR_CRANK", 2, false}, {"FL_THIGH", 3, false}, {"FL_CRANK", 4, false},
  {"RR_THIGH", 7, false}, {"RR_CRANK", 8, false}, {"RL_THIGH", 5, false}, {"RL_CRANK", 6, false},
  {"FL_Z", UNSET, false}, {"FR_Z", UNSET, false}, {"RL_Z", UNSET, false}, {"RR_Z", UNSET, false}
};
const uint8_t MOTOR_COUNT = sizeof(motors) / sizeof(motors[0]);
const uint8_t EXISTING_COUNT = 8;
inline const char* modeName(OperatingMode mode) {
  return mode == CURRENT_8XM ? "CURRENT_8XM" : "FUTURE_12XM";
}
inline uint8_t activeCount(OperatingMode mode) {
  return mode == CURRENT_8XM ? EXISTING_COUNT : MOTOR_COUNT;
}
inline bool allowedId(uint16_t id) {
  // Legacy AX-12A IDs remain excluded even after replacement. No broadcast.
  return id <= 252 && !(id >= 11 && id <= 14);
}
inline bool validConfig(const Motor* list, uint8_t count) {
  for (uint8_t i = 0; i < count; ++i) {
    if (!allowedId(list[i].id)) return false;
    for (uint8_t j = 0; j < i; ++j)
      if (list[j].id == list[i].id) return false;
  }
  return true;
}
inline bool validModeConfig(const Motor* list, uint8_t count, OperatingMode mode) {
  if ((mode != CURRENT_8XM && mode != FUTURE_12XM) || count != activeCount(mode)) return false;
  if (!validConfig(list, count)) return false;
  const uint8_t existingIds[EXISTING_COUNT] = {1, 2, 3, 4, 7, 8, 5, 6};
  for (uint8_t i = 0; i < EXISTING_COUNT; ++i)
    if (list[i].id != existingIds[i]) return false;
  for (uint8_t i = EXISTING_COUNT; i < count; ++i)
    if (!list[i].replacementVerified) return false;
  return true;
}
struct Field { const char* name; uint16_t address; uint8_t width; };
// Explicit XM430-W350 control table. No cached-model or signed-byte decoder.
const Field fields[] = {
  {"model_number", 0, 2}, {"motor_id", 7, 1}, {"firmware_version", 6, 1},
  {"present_position_raw", 132, 4}, {"present_velocity_raw", 128, 4},
  {"torque_enable", 64, 1}, {"operating_mode", 11, 1}, {"drive_mode", 10, 1},
  {"homing_offset_raw", 20, 4}, {"min_position_limit_raw", 52, 4},
  {"max_position_limit_raw", 48, 4}, {"hardware_error_status", 70, 1},
  {"secondary_id", 12, 1}
};
const uint8_t FIELD_COUNT = sizeof(fields) / sizeof(fields[0]);
inline int32_t decode(const uint8_t* b, uint8_t width) {
  uint32_t bits = 0;
  for (uint8_t i = 0; i < width; ++i) bits |= uint32_t(b[i]) << (8 * i);
  int32_t value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}
enum Command { NONE, HELP, LIST, SCAN, SELECT_CURRENT, SELECT_FUTURE, INVALID };
struct Parser {
  char line[16];
  uint8_t length = 0;
  bool discard = false;
  Command feed(char c) {
    if (c == '\r' || c == '\n') {
      line[length] = 0;
      Command result = discard ? INVALID : length == 0 ? NONE :
        strcmp(line, "HELP") == 0 ? HELP : strcmp(line, "LIST") == 0 ? LIST :
        strcmp(line, "SCAN") == 0 ? SCAN :
        strcmp(line, "CURRENT_8XM") == 0 ? SELECT_CURRENT :
        strcmp(line, "FUTURE_12XM") == 0 ? SELECT_FUTURE : INVALID;
      length = 0; discard = false;
      return result;
    }
    if (c < 32 || c > 126 || length >= sizeof(line) - 1) discard = true;
    if (!discard) line[length++] = c;
    return NONE;
  }
};
// Long-form CSV: one register per row, fixed ten columns; no user text echoed.
template<class Output>
void row(Output& out, uint32_t scan, const Motor& motor, const Field* field,
         bool hasValue, int32_t value, const char* status, int libError, int packetError) {
  out.print(scan); out.print(','); out.print(motor.role); out.print(',');
  if (motor.id != UNSET) out.print(motor.id);
  out.print(',');
  if (field) out.print(field->name);
  out.print(','); if (field) out.print(field->address);
  out.print(','); if (field) out.print(field->width);
  out.print(','); if (hasValue) out.print(value);
  out.print(','); out.print(status);
  out.print(','); out.print(libError);
  out.print(','); out.println(packetError);
}
} // namespace Diagnostics
