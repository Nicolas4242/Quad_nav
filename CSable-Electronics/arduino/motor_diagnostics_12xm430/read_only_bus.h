#pragma once
#include <Dynamixel2Arduino.h>
#include "diagnostics.h"

// D2A 0.8.2 Master::read does not check the response ID. Use its protected
// packet builder/parser with a fixed READ instruction and verify the sender.
class ReadOnlyBus : public Dynamixel2Arduino {
 public:
  ReadOnlyBus() : Dynamixel2Arduino(Serial, 2) {}
  int packetError = 0;
  bool wrongSender = false;
  int32_t readRegister(uint16_t id, const Diagnostics::Field& field, uint8_t* bytes) {
    packetError = 0;
    wrongSender = false;
    if (!Diagnostics::allowedId(id) || field.width == 0 || field.width > 4) return -1;
    uint8_t params[4] = {uint8_t(field.address), uint8_t(field.address >> 8), field.width, 0};
    if (!txInstPacket(uint8_t(id), 0x02, params, sizeof(params))) return -1;
    const auto* reply = rxStatusPacket(bytes, 4, 20);
    if (!reply) return -1;
    packetError = reply->err_idx;
    wrongSender = reply->id != id;
    return wrongSender ? -1 : int32_t(reply->recv_param_len);
  }
};
