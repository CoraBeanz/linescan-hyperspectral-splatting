#include "so101_scan_hardware/sts_bus.hpp"

#include <chrono>
#include <set>

namespace so101_scan_hardware
{

void StsBus::open(const std::string & port, int baud_rate, int timeout_ms)
{
  port_.open(port, baud_rate);
  parser_.clear();
  timeout_ms_ = timeout_ms;
}

void StsBus::send(const std::vector<uint8_t> & packet)
{
  // Anything still in the buffer belongs to an earlier exchange.
  port_.flush_input();
  parser_.clear();
  port_.write(packet);
  last_sent_ = packet;
  ++stats_.sent;
}

template<typename Done>
void StsBus::receive(int timeout_ms, Done done)
{
  using Clock = std::chrono::steady_clock;
  const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
  uint8_t buffer[256];
  while (true) {
    while (auto pkt = parser_.next()) {
      // Some adapters echo what the host sends; that isn't an answer.
      if (pkt->raw == last_sent_ || pkt->id == sts::kBroadcastId) {
        continue;
      }
      ++stats_.received;
      last_error_[pkt->id] = pkt->code;
      if (pkt->code != 0) {
        ++stats_.servo_errors;
      }
      if (done(*pkt)) {
        return;
      }
    }
    const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
    if (left <= 0) {
      ++stats_.timeouts;
      return;
    }
    const std::size_t n = port_.read(buffer, sizeof(buffer), static_cast<int>(left));
    parser_.feed(buffer, n);
  }
}

bool StsBus::ping(uint8_t id)
{
  send(sts::make_packet(id, sts::kPing, {}));
  bool ok = false;
  receive(timeout_ms_, [&](const sts::Packet & p) {ok = p.id == id; return ok;});
  return ok;
}

std::optional<std::vector<uint8_t>> StsBus::read(uint8_t id, uint8_t address, uint8_t length)
{
  send(sts::make_read(id, address, length));
  std::optional<std::vector<uint8_t>> out;
  receive(timeout_ms_, [&](const sts::Packet & p) {
      if (p.id == id && p.params.size() == length) {
        out = p.params;
        return true;
      }
      return false;
    });
  return out;
}

std::optional<uint16_t> StsBus::read_u16(uint8_t id, uint8_t address)
{
  auto d = read(id, address, 2);
  if (!d) {
    return std::nullopt;
  }
  return sts::le16(d->data());
}

std::optional<uint8_t> StsBus::read_u8(uint8_t id, uint8_t address)
{
  auto d = read(id, address, 1);
  if (!d) {
    return std::nullopt;
  }
  return (*d)[0];
}

bool StsBus::write(uint8_t id, uint8_t address, const std::vector<uint8_t> & data)
{
  send(sts::make_write(id, address, data));
  bool ok = false;
  receive(timeout_ms_, [&](const sts::Packet & p) {ok = p.id == id; return ok;});
  return ok;
}

void StsBus::sync_write(
  uint8_t address, uint8_t length,
  const std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & data)
{
  send(sts::make_sync_write(address, length, data));
}

std::map<uint8_t, std::vector<uint8_t>> StsBus::sync_read(
  uint8_t address, uint8_t length, const std::vector<uint8_t> & ids)
{
  send(sts::make_sync_read(address, length, ids));
  std::map<uint8_t, std::vector<uint8_t>> out;
  const std::set<uint8_t> wanted(ids.begin(), ids.end());
  receive(timeout_ms_, [&](const sts::Packet & p) {
      if (wanted.count(p.id) && p.params.size() == length) {
        out[p.id] = p.params;
      }
      return out.size() == wanted.size();
    });
  return out;
}

uint8_t StsBus::last_error(uint8_t id) const
{
  auto it = last_error_.find(id);
  return it == last_error_.end() ? 0 : it->second;
}

}  // namespace so101_scan_hardware
