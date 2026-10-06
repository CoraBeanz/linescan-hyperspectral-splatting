#include "so101_scan_hardware/sts_protocol.hpp"

#include <algorithm>
#include <cstdlib>
#include <stdexcept>

namespace so101_scan_hardware
{
namespace sts
{

uint8_t checksum(const uint8_t * begin, const uint8_t * end)
{
  unsigned sum = 0;
  for (const uint8_t * p = begin; p != end; ++p) {
    sum += *p;
  }
  return static_cast<uint8_t>(~sum & 0xFF);
}

std::vector<uint8_t> make_packet(uint8_t id, uint8_t instruction, const std::vector<uint8_t> & params)
{
  if (params.size() > 250) {
    throw std::invalid_argument("STS packet too long");
  }
  std::vector<uint8_t> p = {0xFF, 0xFF, id, static_cast<uint8_t>(params.size() + 2), instruction};
  p.insert(p.end(), params.begin(), params.end());
  p.push_back(checksum(p.data() + 2, p.data() + p.size()));
  return p;
}

std::vector<uint8_t> make_read(uint8_t id, uint8_t address, uint8_t length)
{
  return make_packet(id, kRead, {address, length});
}

std::vector<uint8_t> make_write(uint8_t id, uint8_t address, const std::vector<uint8_t> & data)
{
  std::vector<uint8_t> params = {address};
  params.insert(params.end(), data.begin(), data.end());
  return make_packet(id, kWrite, params);
}

std::vector<uint8_t> make_sync_read(uint8_t address, uint8_t length, const std::vector<uint8_t> & ids)
{
  std::vector<uint8_t> params = {address, length};
  params.insert(params.end(), ids.begin(), ids.end());
  return make_packet(kBroadcastId, kSyncRead, params);
}

std::vector<uint8_t> make_sync_write(
  uint8_t address, uint8_t length,
  const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & data)
{
  std::vector<uint8_t> params = {address, length};
  for (const auto & [id, bytes] : data) {
    if (bytes.size() != length) {
      throw std::invalid_argument("sync write data has the wrong length");
    }
    params.push_back(id);
    params.insert(params.end(), bytes.begin(), bytes.end());
  }
  return make_packet(kBroadcastId, kSyncWrite, params);
}

void PacketParser::feed(const uint8_t * data, std::size_t size)
{
  buffer_.insert(buffer_.end(), data, data + size);
}

std::optional<Packet> PacketParser::next()
{
  while (true) {
    // find the 0xFF 0xFF header
    std::size_t start = 0;
    while (start + 1 < buffer_.size() && !(buffer_[start] == 0xFF && buffer_[start + 1] == 0xFF)) {
      ++start;
    }
    if (start > 0) {
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(start));
    }
    if (buffer_.size() < 4) {
      return std::nullopt;
    }
    // a third 0xFF is not a valid id: drop one and look again
    if (buffer_[2] == 0xFF) {
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t length = buffer_[3];
    if (length < 2) {
      buffer_.erase(buffer_.begin());
      continue;
    }
    const std::size_t total = 4 + length;
    if (buffer_.size() < total) {
      return std::nullopt;
    }
    const uint8_t expected = checksum(buffer_.data() + 2, buffer_.data() + total - 1);
    if (buffer_[total - 1] != expected) {
      ++checksum_errors_;
      buffer_.erase(buffer_.begin());
      continue;
    }
    Packet pkt;
    pkt.id = buffer_[2];
    pkt.code = buffer_[4];
    pkt.params.assign(buffer_.begin() + 5, buffer_.begin() + static_cast<std::ptrdiff_t>(total - 1));
    pkt.raw.assign(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total));
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(total));
    return pkt;
  }
}

int decode_signed(uint16_t raw, int sign_bit)
{
  const uint16_t mask = static_cast<uint16_t>(1u << sign_bit);
  const int magnitude = raw & (mask - 1);
  return (raw & mask) ? -magnitude : magnitude;
}

uint16_t encode_signed(int value, int sign_bit)
{
  const int limit = (1 << sign_bit) - 1;
  const int magnitude = std::min(std::abs(value), limit);
  return static_cast<uint16_t>(magnitude | (value < 0 ? (1 << sign_bit) : 0));
}

}  // namespace sts
}  // namespace so101_scan_hardware
