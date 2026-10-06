#include "so101_scan_hardware/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace so101_scan_hardware
{
namespace
{

speed_t to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default:
      throw std::runtime_error("unsupported baud rate " + std::to_string(baud));
  }
}

std::string errno_text() {return std::strerror(errno);}

}  // namespace

SerialPort::~SerialPort() {close();}

void SerialPort::open(const std::string & path, int baud_rate)
{
  close();
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("can't open " + path + ": " + errno_text());
  }
  path_ = path;
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    const std::string why = errno_text();
    close();
    throw std::runtime_error("can't read the settings of " + path + ": " + why);
  }
  cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~(CSTOPB | CRTSCTS);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  const speed_t speed = to_speed(baud_rate);
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    const std::string why = errno_text();
    close();
    throw std::runtime_error("can't configure " + path + ": " + why);
  }
  flush_input();
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SerialPort::write(const std::vector<uint8_t> & data)
{
  std::size_t done = 0;
  while (done < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + done, data.size() - done);
    if (n > 0) {
      done += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EINTR) {
      throw std::runtime_error("write to " + path_ + " failed: " + errno_text());
    }
    pollfd p{fd_, POLLOUT, 0};
    if (::poll(&p, 1, 100) <= 0) {
      throw std::runtime_error("write to " + path_ + " timed out");
    }
  }
}

std::size_t SerialPort::read(uint8_t * buffer, std::size_t max, int timeout_ms)
{
  pollfd p{fd_, POLLIN, 0};
  const int ready = ::poll(&p, 1, timeout_ms);
  if (ready < 0) {
    if (errno == EINTR) {
      return 0;
    }
    throw std::runtime_error("poll on " + path_ + " failed: " + errno_text());
  }
  if (ready == 0) {
    return 0;
  }
  if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    throw std::runtime_error(path_ + " was disconnected");
  }
  const ssize_t n = ::read(fd_, buffer, max);
  if (n < 0) {
    if (errno == EAGAIN || errno == EINTR) {
      return 0;
    }
    throw std::runtime_error("read from " + path_ + " failed: " + errno_text());
  }
  return static_cast<std::size_t>(n);
}

void SerialPort::flush_input()
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIFLUSH);
  }
}

}  // namespace so101_scan_hardware
