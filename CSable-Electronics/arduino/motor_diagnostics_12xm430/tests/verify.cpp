#include <iostream>
#include <algorithm>
#include "../motor_diagnostics_12xm430.ino"

void clearOutput() { Serial2.output.str(""); Serial2.output.clear(); }
void command(const std::string& text) {
  Serial2.input = text;
  while (Serial2.available()) loop();
}
int main() {
  using namespace Diagnostics;
  assert(validModeConfig(motors, EXISTING_COUNT, CURRENT_8XM));
  assert(!validModeConfig(motors, MOTOR_COUNT, FUTURE_12XM));
  for (unsigned id = 0; id <= 65535; ++id)
    assert(allowedId(id) == (id <= 252 && !(id >= 11 && id <= 14)));
  Motor test[] = {{"A", 1, false}, {"B", 1, false}};
  assert(!validConfig(test, 2));
  for (auto id : {11, 12, 13, 14, 253, 254, 256, 65535}) {
    test[1].id = id; assert(!validConfig(test, 2));
  }
  test[0].id = UNSET; test[1].id = UNSET; assert(!validConfig(test, 2));
  setup(); loop(); assert(dxl.requests.empty());
  assert(operatingMode == CURRENT_8XM);
  assert(Serial2.output.str().find("# MODE,CURRENT_8XM,configuration_valid=1") != std::string::npos);
  command("FUTURE_12XM\r\n"); assert(dxl.requests.empty());
  assert(operatingMode == FUTURE_12XM);
  command("SCAN\n"); assert(dxl.requests.empty());
  assert(scanNumber == 0);
  command("CURRENT_8XM\n"); assert(dxl.requests.empty());
  command("HELP\r\nLIST\nscan\n SCAN\nSCAN 1\nSTOP\n");
  command(std::string(30, 'X') + "SCAN\n");
  command(std::string("SC\0AN\n", 6));
  assert(dxl.requests.empty());
  command("SCAN"); assert(dxl.requests.empty());
  clearOutput(); command("\r\n");
  assert(dxl.requests.size() == 8 * FIELD_COUNT);
  for (const auto& request : dxl.requests) assert(request.id >= 1 && request.id <= 8);
  const std::string csv = Serial2.output.str();
  assert(csv.find("1,FR_THIGH,1,firmware_version,6,1,200,ok,0,0\n") != std::string::npos);
  assert(csv.find("present_position_raw,132,4,-2147483648,ok") != std::string::npos);
  assert(csv.find("present_velocity_raw,128,4,-17,ok") != std::string::npos);
  assert(csv.find("homing_offset_raw,20,4,-1024,ok") != std::string::npos);
  assert(csv.find("secondary_id,12,1,255,ok") != std::string::npos);
  assert(csv.find("FL_Z") == std::string::npos);
  assert(csv.find("# SCAN_MODE,CURRENT_8XM") != std::string::npos);
  std::istringstream lines(csv); std::string line; unsigned rows = 0;
  while (std::getline(lines, line)) {
    if (line[0] == '#') continue;
    assert(std::count(line.begin(), line.end(), ',') == 9); ++rows;
  }
  assert(rows == 1 + 8 * FIELD_COUNT);
  for (int fault = 1; fault <= 7; ++fault) {
    dxl.fault = fault; dxl.requests.clear(); clearOutput(); command("SCAN\n");
    assert(dxl.requests.size() == (fault == 4 ? 8 * FIELD_COUNT : fault == 6 ? 16 : 8));
    std::string out = Serial2.output.str();
    const char* expected[] = {"", "transport_error", "length_error", "status_error",
                              "alert,0,128", "unexpected_model", "id_mismatch", "wrong_response_id"};
    assert(out.find(expected[fault]) != std::string::npos);
    assert(out.find("# END_SCAN") != std::string::npos);
  }
  dxl.fault = 8; dxl.requests.clear(); clearOutput(); command("SCAN\n");
  assert(dxl.requests.size() == 8 * FIELD_COUNT);
  assert(Serial2.output.str().find("present_position_raw,132,4,,status_error,0,4") != std::string::npos);
  assert(Serial2.output.str().find("secondary_id,12,1,255,ok") != std::string::npos);
  dxl.fault = 9; dxl.requests.clear(); clearOutput(); command("SCAN\n");
  assert(dxl.requests.empty());
  assert(Serial2.output.str().find("transport_error") != std::string::npos);
  dxl.fault = 10; dxl.requests.clear(); clearOutput(); command("SCAN\n");
  assert(dxl.requests.size() == 8);
  assert(Serial2.output.str().find("length_error") != std::string::npos);
  dxl.fault = 11; dxl.requests.clear(); clearOutput(); command("SCAN\n");
  assert(dxl.requests.size() == 8);
  assert(Serial2.output.str().find("model_number,0,2,,status_error,0,4") != std::string::npos);
  dxl.requests.clear(); uint8_t bytes[4]{};
  for (auto id : {11, 12, 13, 14, 253, 254, 255, 256, 65535})
    assert(dxl.readRegister(id, fields[0], bytes) == -1);
  assert(dxl.requests.empty());
  // Synthetic host-only IDs: never copied into the uploadable configuration.
  Motor future[MOTOR_COUNT];
  std::copy(motors, motors + MOTOR_COUNT, future);
  for (uint8_t i = EXISTING_COUNT; i < MOTOR_COUNT; ++i) future[i].id = 21 + i - EXISTING_COUNT;
  dxl.fault = 0;
  scanConfigured(future, MOTOR_COUNT, FUTURE_12XM);
  assert(dxl.requests.empty()); // IDs alone cannot authorize replacement reads.
  for (uint8_t i = EXISTING_COUNT; i < MOTOR_COUNT; ++i) future[i].replacementVerified = true;
  assert(validModeConfig(future, MOTOR_COUNT, FUTURE_12XM));
  clearOutput(); scanConfigured(future, MOTOR_COUNT, FUTURE_12XM);
  assert(dxl.requests.size() == 12 * FIELD_COUNT);
  assert(Serial2.output.str().find("# SCAN_MODE,FUTURE_12XM") != std::string::npos);
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i)
    for (uint8_t f = 0; f < FIELD_COUNT; ++f)
      assert(dxl.requests[i * FIELD_COUNT + f].id == future[i].id);
  for (int fault = 1; fault <= 11; ++fault) {
    dxl.fault = fault; dxl.requests.clear(); clearOutput();
    scanConfigured(future, MOTOR_COUNT, FUTURE_12XM);
    const unsigned expected = fault == 9 ? 0 : fault == 4 || fault == 8 ? 12 * FIELD_COUNT : fault == 6 ? 24 : 12;
    assert(dxl.requests.size() == expected);
    assert(Serial2.output.str().find("# END_SCAN") != std::string::npos);
    if (fault == 11)
      assert(Serial2.output.str().find("model_number,0,2,,status_error,0,4") != std::string::npos);
  }
  dxl.fault = 0;
  dxl.requests.clear();
  for (uint8_t i = EXISTING_COUNT; i < MOTOR_COUNT; ++i) {
    uint16_t original = future[i].id;
    for (auto bad : {1, 11, 12, 13, 14, 253, 254, 255, 256, 65535}) {
      future[i].id = bad;
      scanConfigured(future, MOTOR_COUNT, FUTURE_12XM);
      assert(dxl.requests.empty());
    }
    future[i].id = original;
    future[i].replacementVerified = false;
    scanConfigured(future, MOTOR_COUNT, FUTURE_12XM); assert(dxl.requests.empty());
    future[i].replacementVerified = true;
  }
  future[9].id = future[8].id;
  scanConfigured(future, MOTOR_COUNT, FUTURE_12XM); assert(dxl.requests.empty());
  // Current mode ignores all replacement entries, even invalid ones.
  scanConfigured(future, EXISTING_COUNT, CURRENT_8XM);
  assert(dxl.requests.size() == 8 * FIELD_COUNT);
  dxl.requests.clear();
  future[0].id = 9;
  scanConfigured(future, EXISTING_COUNT, CURRENT_8XM); assert(dxl.requests.empty());
  assert(!validModeConfig(motors, MOTOR_COUNT, CURRENT_8XM));
  assert(!validModeConfig(motors, EXISTING_COUNT, FUTURE_12XM));
  dxl.fault = 0; busReady = false; dxl.requests.clear(); command("SCAN\n");
  assert(dxl.requests.empty());
  assert(Serial.output.str().empty());
  std::cout << "PASS: both modes, future verification gates, ID policy, parser, CSV, signed decoding, read-only API, identity gates, faults and boot\n";
}
