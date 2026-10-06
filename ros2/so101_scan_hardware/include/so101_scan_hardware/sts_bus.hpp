// Request/response over the STS servo bus: ping, read, write, and the sync
// read/write that talk to every servo in one go.
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "so101_scan_hardware/serial_port.hpp"
#include "so101_scan_hardware/sts_protocol.hpp"

namespace so101_scan_hardware
{

class StsBus
{
public:
  struct Stats
  {
    uint64_t sent = 0;
    uint64_t received = 0;
    uint64_t timeouts = 0;      // a servo that didn't answer in time
    uint64_t servo_errors = 0;  // status packets with error bits set
  };

  void open(const std::string & port, int baud_rate, int timeout_ms);
  void close() {port_.close();}
  bool is_open() const {return port_.is_open();}

  bool ping(uint8_t id);
  std::optional<std::vector<uint8_t>> read(uint8_t id, uint8_t address, uint8_t length);
  bool write(uint8_t id, uint8_t address, const std::vector<uint8_t> & data);
  bool write_u8(uint8_t id, uint8_t address, uint8_t value) {return write(id, address, {value});}
  bool write_u16(uint8_t id, uint8_t address, uint16_t value) {return write(id, address, sts::to_le16(value));}
  std::optional<uint16_t> read_u16(uint8_t id, uint8_t address);
  std::optional<uint8_t> read_u8(uint8_t id, uint8_t address);

  // The servos don't answer a sync write.
  void sync_write(
    uint8_t address, uint8_t length,
    const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & data);
  // Each servo answers in turn; servos that don't answer in time are missing
  // from the result.
  std::map<uint8_t, std::vector<uint8_t>> sync_read(
    uint8_t address, uint8_t length, const std::vector<uint8_t> & ids);

  // Error bits from the last status packet each servo sent.
  uint8_t last_error(uint8_t id) const;
  const Stats & stats() const {return stats_;}

private:
  void send(const std::vector<uint8_t> & packet);
  // Waits for status packets until `done` says stop or the timeout runs out.
  template<typename Done>
  void receive(int timeout_ms, Done done);

  SerialPort port_;
  sts::PacketParser parser_;
  std::vector<uint8_t> last_sent_;
  std::map<uint8_t, uint8_t> last_error_;
  int timeout_ms_ = 20;
  Stats stats_;
};

}  // namespace so101_scan_hardware
