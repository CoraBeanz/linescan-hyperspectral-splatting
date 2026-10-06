// Feetech STS serial bus protocol (the STS3215 servos in the SO-101).
//
// Every packet is  0xFF 0xFF  ID  LEN  INSTR|ERROR  PARAMS...  CHECKSUM
// where LEN counts the bytes after itself (params + 2) and CHECKSUM is the
// inverted low byte of ID + LEN + INSTR + PARAMS. The host sends instruction
// packets; a servo answers with a status packet whose fourth byte is an error
// bitfield instead of an instruction. Two-byte registers are little-endian.
//
// These are pure functions with no I/O, so the tests can check them byte for
// byte; StsBus does the talking.
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace so101_scan_hardware
{
namespace sts
{

constexpr uint8_t kBroadcastId = 0xFE;
constexpr int kTicksPerTurn = 4096;
constexpr uint16_t kStsModelNumber = 777;  // STS3215

enum Instruction : uint8_t
{
  kPing = 0x01,
  kRead = 0x02,
  kWrite = 0x03,
  kSyncRead = 0x82,
  kSyncWrite = 0x83,
};

// STS3215 control table: register addresses (sizes in the comments).
namespace reg
{
constexpr uint8_t kModelNumber = 3;         // 2
constexpr uint8_t kId = 5;                  // 1
constexpr uint8_t kReturnDelayTime = 7;     // 1, units of 2 us
constexpr uint8_t kMinPositionLimit = 9;    // 2
constexpr uint8_t kMaxPositionLimit = 11;   // 2
constexpr uint8_t kPCoefficient = 21;       // 1
constexpr uint8_t kDCoefficient = 22;       // 1
constexpr uint8_t kICoefficient = 23;       // 1
constexpr uint8_t kHomingOffset = 31;       // 2, sign bit 11
constexpr uint8_t kOperatingMode = 33;      // 1, 0 = position
constexpr uint8_t kTorqueEnable = 40;       // 1
constexpr uint8_t kAcceleration = 41;       // 1
constexpr uint8_t kGoalPosition = 42;       // 2
constexpr uint8_t kGoalVelocity = 46;       // 2, sign bit 15
constexpr uint8_t kLock = 55;               // 1, 0 lets EEPROM writes stick
constexpr uint8_t kPresentPosition = 56;    // 2
constexpr uint8_t kPresentVelocity = 58;    // 2, sign bit 15, ticks/s
constexpr uint8_t kPresentLoad = 60;        // 2, sign bit 10, 0.1 % of max torque
constexpr uint8_t kPresentVoltage = 62;     // 1, 0.1 V
constexpr uint8_t kPresentTemperature = 63; // 1, deg C
constexpr uint8_t kMoving = 66;             // 1
}  // namespace reg

// Error bits in a status packet.
namespace error
{
constexpr uint8_t kVoltage = 0x01;
constexpr uint8_t kAngle = 0x02;
constexpr uint8_t kOverheat = 0x04;
constexpr uint8_t kOverEle = 0x08;
constexpr uint8_t kOverload = 0x20;
}  // namespace error

struct Packet
{
  uint8_t id = 0;
  uint8_t code = 0;  // instruction (host to servo) or error bits (servo to host)
  std::vector<uint8_t> params;
  std::vector<uint8_t> raw;  // the whole packet as received
};

uint8_t checksum(const uint8_t * begin, const uint8_t * end);

std::vector<uint8_t> make_packet(uint8_t id, uint8_t instruction, const std::vector<uint8_t> & params);
std::vector<uint8_t> make_read(uint8_t id, uint8_t address, uint8_t length);
std::vector<uint8_t> make_write(uint8_t id, uint8_t address, const std::vector<uint8_t> & data);
std::vector<uint8_t> make_sync_read(uint8_t address, uint8_t length, const std::vector<uint8_t> & ids);
// Every entry's data must be `length` bytes long.
std::vector<uint8_t> make_sync_write(
  uint8_t address, uint8_t length,
  const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & data);

// Splits a byte stream into packets. Bytes that don't form a packet with a
// valid checksum are skipped, so it resynchronises after line noise.
class PacketParser
{
public:
  void feed(const uint8_t * data, std::size_t size);
  std::optional<Packet> next();
  void clear() {buffer_.clear();}
  std::size_t checksum_errors() const {return checksum_errors_;}

private:
  std::vector<uint8_t> buffer_;
  std::size_t checksum_errors_ = 0;
};

inline uint16_t le16(const uint8_t * p) {return static_cast<uint16_t>(p[0] | (p[1] << 8));}
inline std::vector<uint8_t> to_le16(uint16_t v)
{
  return {static_cast<uint8_t>(v & 0xFF), static_cast<uint8_t>(v >> 8)};
}

// Feetech's signed registers keep the magnitude in the low bits and the sign
// in one bit (bit 15 for speeds, 11 for the homing offset, 10 for the load).
int decode_signed(uint16_t raw, int sign_bit);
uint16_t encode_signed(int value, int sign_bit);

}  // namespace sts
}  // namespace so101_scan_hardware
