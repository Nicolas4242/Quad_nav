#include <Dynamixel2Arduino.h>
#include "diagnostics.h"
#include "read_only_bus.h"

// Mega + ROBOTIS Shield: UART0 is exclusively the motor bus, never a console.
ReadOnlyBus dxl;
Diagnostics::Parser parser;
uint32_t scanNumber = 0;
bool busReady = false;
Diagnostics::OperatingMode operatingMode = Diagnostics::CURRENT_8XM;

void printMode() {
  Serial2.print(F("# MODE,")); Serial2.print(Diagnostics::modeName(operatingMode));
  Serial2.print(F(",configuration_valid="));
  Serial2.println(Diagnostics::validModeConfig(Diagnostics::motors,
    Diagnostics::activeCount(operatingMode), operatingMode) ? 1 : 0);
}

void printHelp() {
  Serial2.println(F("# READ ONLY: current robot = 8 XM430 + 4 AX-12A; target = 12 XM430"));
  Serial2.println(F("# Future mode requires four verified replacement IDs; legacy IDs 11-14 always excluded"));
  Serial2.println(F("# Mode commands: CURRENT_8XM FUTURE_12XM. Selection sends no reads. Protocol 2.0 only."));
  Serial2.println(F("# Exact commands + newline: HELP LIST SCAN. No automatic scan or torque changes."));
  printMode();
}

void printList() {
  printMode();
  for (uint8_t i = 0; i < Diagnostics::activeCount(operatingMode); ++i) {
    Serial2.print(F("# CONFIG,")); Serial2.print(Diagnostics::motors[i].role);
    Serial2.print(',');
    if (Diagnostics::motors[i].id == Diagnostics::UNSET) Serial2.println(F("UNSET"));
    else Serial2.println(Diagnostics::motors[i].id);
    if (i >= Diagnostics::EXISTING_COUNT) {
      Serial2.print(F("# replacement_verified,"));
      Serial2.println(Diagnostics::motors[i].replacementVerified ? 1 : 0);
    }
  }
}

void scanConfigured(const Diagnostics::Motor* list, uint8_t count, Diagnostics::OperatingMode mode) {
  using namespace Diagnostics;
  Serial2.print(F("# SCAN_MODE,")); Serial2.println(modeName(mode));
  if (!busReady || !validModeConfig(list, count, mode)) {
    Serial2.println(F("# ERROR: invalid configuration or protocol initialization; no reads"));
    return;
  }
  ++scanNumber;
  Serial2.println(F("scan,role,requested_id,field,address,width,value,status,library_error,packet_error"));
  for (uint8_t i = 0; i < count; ++i) {
    const Motor& motor = list[i];
    // Model and primary ID must validate before any further table reads.
    for (uint8_t f = 0; f < FIELD_COUNT; ++f) {
      const Field& field = fields[f];
      uint8_t bytes[4] = {0, 0, 0, 0};
      const int32_t received = dxl.readRegister(motor.id, field, bytes);
      const int libError = dxl.getLastLibErrCode();
      const int packetError = dxl.packetError;
      // An alert-only status (bit 7) still has usable data. Instruction errors do not.
      bool usable = received == field.width && libError == 0 && (packetError & 0x7f) == 0;
      int32_t value = usable ? decode(bytes, field.width) : 0;
      const char* status = dxl.wrongSender ? "wrong_response_id" :
        libError != 0 ? "transport_error" :
        // A valid motor error reply may have no data; retain its actual cause.
        (packetError & 0x7f) != 0 ? "status_error" :
        received != field.width ? "length_error" :
        packetError != 0 ? "alert" : "ok";
      bool identityOK = usable && (f != 0 || value == 1020) && (f != 1 || value == motor.id);
      if (usable && f == 0 && value != 1020) status = "unexpected_model";
      if (usable && f == 1 && value != motor.id) status = "id_mismatch";
      row(Serial2, scanNumber, motor, &field, usable, value, status, libError, packetError);
      if (f < 2 && !identityOK) {
        row(Serial2, scanNumber, motor, nullptr, false, 0, "identity_failed_remaining_skipped", 0, 0);
        break;
      }
    }
  }
  Serial2.println(F("# END_SCAN"));
}

void scanMotors() {
  scanConfigured(Diagnostics::motors, Diagnostics::activeCount(operatingMode), operatingMode);
}

void setup() {
  Serial2.begin(115200);
  dxl.begin(1000000);
  busReady = dxl.setPortProtocolVersion(2.0f); // Local parser setting, not a motor write.
  printHelp();
  printList();
}

void loop() {
  // Bounded input work; a scan itself has a fixed number of bounded read attempts.
  for (uint8_t n = 0; n < 32 && Serial2.available() > 0; ++n) {
    switch (parser.feed(char(Serial2.read()))) {
      case Diagnostics::HELP: printHelp(); break;
      case Diagnostics::LIST: printList(); break;
      case Diagnostics::SCAN: scanMotors(); break;
      case Diagnostics::SELECT_CURRENT: operatingMode = Diagnostics::CURRENT_8XM; printList(); break;
      case Diagnostics::SELECT_FUTURE: operatingMode = Diagnostics::FUTURE_12XM; printList(); break;
      case Diagnostics::INVALID: Serial2.println(F("# ERROR: command rejected")); break;
      default: break;
    }
  }
}
