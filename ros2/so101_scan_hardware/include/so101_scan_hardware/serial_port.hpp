// A raw serial port on Linux (termios), for the servo bus adapter.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace so101_scan_hardware
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Opens and configures the port: raw 8N1, no flow control. Throws
  // std::runtime_error with the reason if it can't. A pseudo-terminal (the
  // fake bus in the tests) accepts any baud rate.
  void open(const std::string & path, int baud_rate);
  void close();
  bool is_open() const {return fd_ >= 0;}
  const std::string & path() const {return path_;}

  // Writes everything or throws.
  void write(const std::vector<uint8_t> & data);
  // Reads whatever arrives within timeout_ms (up to max bytes). Returns the
  // number of bytes read, 0 on timeout.
  std::size_t read(uint8_t * buffer, std::size_t max, int timeout_ms);
  // Drops anything waiting in the input buffer.
  void flush_input();

private:
  int fd_ = -1;
  std::string path_;
};

}  // namespace so101_scan_hardware
