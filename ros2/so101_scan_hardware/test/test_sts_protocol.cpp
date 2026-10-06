#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "so101_scan_hardware/feetech_sts_system.hpp"
#include "so101_scan_hardware/sts_protocol.hpp"

using so101_scan_hardware::FeetechStsSystem;
namespace sts = so101_scan_hardware::sts;
using Bytes = std::vector<uint8_t>;

TEST(StsProtocol, PingPacket)
{
  EXPECT_EQ(sts::make_packet(1, sts::kPing, {}), (Bytes{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));
}

TEST(StsProtocol, ReadPresentPosition)
{
  // read 2 bytes at 56 (0x38) from servo 1
  EXPECT_EQ(sts::make_read(1, sts::reg::kPresentPosition, 2),
    (Bytes{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE}));
}

TEST(StsProtocol, SyncWriteGoals)
{
  const auto p = sts::make_sync_write(
    sts::reg::kGoalPosition, 2, {{1, sts::to_le16(2048)}, {2, sts::to_le16(1000)}});
  const Bytes expect_body = {0xFE, 10, 0x83, 42, 2, 1, 0x00, 0x08, 2, 0xE8, 0x03};
  ASSERT_EQ(p.size(), 2 + expect_body.size() + 1);
  EXPECT_EQ(Bytes(p.begin() + 2, p.end() - 1), expect_body);
  EXPECT_EQ(p.back(), sts::checksum(expect_body.data(), expect_body.data() + expect_body.size()));
  EXPECT_THROW(sts::make_sync_write(42, 2, {{1, Bytes{1}}}), std::invalid_argument);
}

TEST(StsProtocol, SyncRead)
{
  const auto p = sts::make_sync_read(56, 8, {1, 2, 3});
  EXPECT_EQ(Bytes(p.begin(), p.end() - 1), (Bytes{0xFF, 0xFF, 0xFE, 7, 0x82, 56, 8, 1, 2, 3}));
}

TEST(StsProtocol, ParserFindsPacketsInNoise)
{
  sts::PacketParser parser;
  const Bytes status = sts::make_packet(3, 0, {0x00, 0x08});  // servo 3 answers 2048
  Bytes stream = {0x12, 0xFF, 0x00};                            // junk, half a header
  stream.insert(stream.end(), status.begin(), status.end());
  Bytes broken = status;
  broken.back() ^= 0x55;                                        // bad checksum
  stream.insert(stream.end(), broken.begin(), broken.end());
  stream.insert(stream.end(), status.begin(), status.begin() + 4);  // first half of another

  parser.feed(stream.data(), stream.size());
  auto pkt = parser.next();
  ASSERT_TRUE(pkt.has_value());
  EXPECT_EQ(pkt->id, 3);
  EXPECT_EQ(pkt->code, 0);
  EXPECT_EQ(sts::le16(pkt->params.data()), 2048);
  EXPECT_FALSE(parser.next().has_value());
  EXPECT_EQ(parser.checksum_errors(), 1u);

  parser.feed(status.data() + 4, status.size() - 4);  // the rest arrives
  pkt = parser.next();
  ASSERT_TRUE(pkt.has_value());
  EXPECT_EQ(pkt->raw, status);
}

TEST(StsProtocol, SignMagnitude)
{
  EXPECT_EQ(sts::decode_signed(0x8000 | 300, 15), -300);
  EXPECT_EQ(sts::decode_signed(300, 15), 300);
  EXPECT_EQ(sts::decode_signed(0x0400 | 125, 10), -125);
  EXPECT_EQ(sts::encode_signed(-1500, 11), 0x0800 | 1500);
  EXPECT_EQ(sts::encode_signed(5000, 11), 2047);
  for (int v : {-2047, -1, 0, 1, 2047}) {
    EXPECT_EQ(sts::decode_signed(sts::encode_signed(v, 11), 11), v);
  }
}

TEST(Units, TicksAndRadians)
{
  FeetechStsSystem::Joint j;
  j.zero_ticks = 2000;
  j.sign = -1;
  j.min_ticks = 1000;
  j.max_ticks = 3000;
  EXPECT_NEAR(FeetechStsSystem::ticks_to_rad(j, 2000), 0.0, 1e-12);
  EXPECT_NEAR(FeetechStsSystem::ticks_to_rad(j, 1000), M_PI / 2 * 1000 / 1024, 1e-12);
  EXPECT_EQ(FeetechStsSystem::rad_to_ticks(j, 0.0), 2000);
  EXPECT_EQ(FeetechStsSystem::rad_to_ticks(j, FeetechStsSystem::ticks_to_rad(j, 1234)), 1234);
  // goals stay within the register's range; the calibrated range is limits(), in rad
  EXPECT_EQ(FeetechStsSystem::rad_to_ticks(j, 10.0), 0);
  EXPECT_EQ(FeetechStsSystem::rad_to_ticks(j, -10.0), 4095);
  const auto [lo, hi] = FeetechStsSystem::limits(j);  // sign -1: max_ticks is the lower end
  EXPECT_NEAR(lo, FeetechStsSystem::ticks_to_rad(j, 3000), 1e-12);
  EXPECT_NEAR(hi, FeetechStsSystem::ticks_to_rad(j, 1000), 1e-12);
}
