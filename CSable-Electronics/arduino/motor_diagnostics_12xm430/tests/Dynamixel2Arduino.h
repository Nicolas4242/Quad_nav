#pragma once
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>
#include <cassert>
#define F(s) s
struct MockSerial {
  std::string input;
  std::ostringstream output;
  void begin(unsigned long) {}
  int available() { return int(input.size()); }
  int read() { char c = input.front(); input.erase(0, 1); return c; }
  template<class T> void print(T v) { output << v; }
  void print(uint8_t v) { output << unsigned(v); }
  template<class T> void println(T v) { print(v); output << '\n'; }
};
MockSerial Serial, Serial2;
struct Request { uint8_t id; uint16_t address; uint16_t width; };
// Intentionally no motor-write API: adding one to the sketch breaks this build.
class Dynamixel2Arduino {
 public:
  std::vector<Request> requests;
  int fault = 0, libError = 0, statusError = 0;
  float protocol = 0;
  Dynamixel2Arduino(MockSerial&, int pin) { assert(pin == 2); }
  void begin(unsigned long baud) { assert(baud == 1000000); }
  bool setPortProtocolVersion(float p) { protocol = p; return p == 2.0f; }
  int getLastLibErrCode() { return libError; }
  int getLastStatusPacketError() { return statusError; }
  int read(uint8_t id, uint16_t addr, uint16_t width, uint8_t* data,
           uint16_t capacity, uint32_t timeout) {
    assert(protocol == 2.0f && id <= 252 && !(id >= 11 && id <= 14));
    assert(width <= capacity && timeout == 20);
    requests.push_back({id, addr, width});
    libError = fault == 1 ? 3 : 0;
    statusError = fault == 3 || fault == 11 || (fault == 8 && addr == 132) ? 4 : fault == 4 ? 128 : 0;
    uint32_t value = addr == 0 ? (fault == 5 ? 12 : 1020) :
      addr == 7 ? (fault == 6 ? id + 1 : id) :
      addr == 6 ? 200 : addr == 12 ? 255 :
      addr == 132 ? uint32_t(-2147483647 - 1) : addr == 128 ? uint32_t(-17) :
      addr == 20 ? uint32_t(-1024) : 0;
    for (uint16_t n = 0; n < width; ++n) data[n] = uint8_t(value >> (8 * n));
    return fault == 1 || fault == 11 ? 0 : fault == 2 ? width - 1 : fault == 10 ? width + 1 : width;
  }
 protected:
  Request pending{};
  struct Reply { uint8_t id; uint8_t err_idx; uint16_t recv_param_len; } reply{};
  bool txInstPacket(uint8_t id, uint8_t instruction, uint8_t* params, uint16_t length) {
    assert(instruction == 0x02 && length == 4); // Only READ is permitted.
    if (fault == 9) { libError = 5; return false; }
    pending = {id, uint16_t(params[0] | (params[1] << 8)), uint16_t(params[2] | (params[3] << 8))};
    return true;
  }
  const Reply* rxStatusPacket(uint8_t* bytes, uint16_t capacity, uint32_t timeout) {
    int count = read(pending.id, pending.address, pending.width, bytes, capacity, timeout);
    if (fault == 1) return nullptr;
    reply = {uint8_t(fault == 7 ? pending.id + 1 : pending.id), uint8_t(statusError), uint16_t(count)};
    return &reply;
  }
};
