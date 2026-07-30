// Copyright 2026 gongbang
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include "rf_joy/ibus_protocol.hpp"

namespace
{
using rf_joy::ibus::kChannelCount;
using rf_joy::ibus::kCommandByte;
using rf_joy::ibus::kLengthByte;
using rf_joy::ibus::kPacketSize;

/// Builds a well formed frame carrying the given raw 16 bit channel words. Words wider than
/// 12 bits are written verbatim so the masking behaviour can be exercised.
std::vector<std::uint8_t> make_frame(const std::vector<std::uint16_t> & raw_channels)
{
  std::vector<std::uint8_t> frame(kPacketSize, 0);
  frame[0] = kLengthByte;
  frame[1] = kCommandByte;

  for (std::size_t i = 0; i < kChannelCount; ++i) {
    const std::uint16_t value = i < raw_channels.size() ? raw_channels[i] : 1500;
    frame[2 + (i * 2)] = static_cast<std::uint8_t>(value & 0xFF);
    frame[3 + (i * 2)] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
  }

  std::uint16_t checksum = 0xFFFF;
  for (std::size_t i = 0; i < kPacketSize - 2; ++i) {
    checksum = static_cast<std::uint16_t>(checksum - frame[i]);
  }
  frame[kPacketSize - 2] = static_cast<std::uint8_t>(checksum & 0xFF);
  frame[kPacketSize - 1] = static_cast<std::uint8_t>((checksum >> 8) & 0xFF);

  return frame;
}

/// Collects every frame the scan reports, so tests can assert on arrival order.
std::vector<std::vector<std::uint16_t>> collect(
  const std::vector<std::uint8_t> & buffer,
  rf_joy::ibus::ScanResult & result)
{
  std::vector<std::vector<std::uint16_t>> decoded;
  result = rf_joy::ibus::scan_frames(
    buffer,
    [&decoded](const std::uint8_t * frame) {
      std::vector<std::uint16_t> channels;
      rf_joy::ibus::decode_channels(frame, channels);
      decoded.push_back(channels);
    });
  return decoded;
}
}  // namespace

TEST(IbusChecksum, AcceptsWellFormedFrame)
{
  const auto frame = make_frame({1000, 1500, 2000});
  EXPECT_TRUE(rf_joy::ibus::verify_checksum(frame.data()));
}

TEST(IbusChecksum, RejectsCorruptedPayload)
{
  auto frame = make_frame({1000, 1500, 2000});
  frame[5] = static_cast<std::uint8_t>(frame[5] ^ 0xFF);
  EXPECT_FALSE(rf_joy::ibus::verify_checksum(frame.data()));
}

TEST(IbusChecksum, RejectsCorruptedChecksumBytes)
{
  auto frame = make_frame({1000, 1500, 2000});
  frame[kPacketSize - 1] = static_cast<std::uint8_t>(frame[kPacketSize - 1] + 1);
  EXPECT_FALSE(rf_joy::ibus::verify_checksum(frame.data()));
}

TEST(IbusDecode, RoundTripsChannelValues)
{
  const std::vector<std::uint16_t> expected{1000, 1100, 1200, 1300, 1400,
    1500, 1600, 1700, 1800, 1900, 2000, 1234, 1543, 1876};
  const auto frame = make_frame(expected);

  std::vector<std::uint16_t> channels;
  rf_joy::ibus::decode_channels(frame.data(), channels);

  ASSERT_EQ(channels.size(), kChannelCount);
  EXPECT_EQ(channels, expected);
}

// With the transmitter in 18 channel mode the extra channels ride in the upper nibbles.
TEST(IbusDecode, MasksUpperNibbleFromExtendedChannelMode)
{
  std::vector<std::uint16_t> raw(kChannelCount, 1500);
  raw[6] = static_cast<std::uint16_t>(0xA000 | 1234);
  raw[13] = static_cast<std::uint16_t>(0x7000 | 1876);
  const auto frame = make_frame(raw);

  std::vector<std::uint16_t> channels;
  rf_joy::ibus::decode_channels(frame.data(), channels);

  EXPECT_EQ(channels[6], 1234);
  EXPECT_EQ(channels[13], 1876);
  EXPECT_EQ(channels[0], 1500);
}

TEST(IbusScan, FindsSingleAlignedFrame)
{
  const auto frame = make_frame({1001, 1002, 1003});
  rf_joy::ibus::ScanResult result;
  const auto decoded = collect(frame, result);

  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0][0], 1001);
  EXPECT_EQ(result.valid_frames, 1U);
  EXPECT_EQ(result.discarded_bytes, 0U);
  EXPECT_EQ(result.consumed, kPacketSize);
}

TEST(IbusScan, FindsBackToBackFramesInArrivalOrder)
{
  std::vector<std::uint8_t> buffer;
  for (std::uint16_t value : {1111, 1222, 1333}) {
    const auto frame = make_frame({value});
    buffer.insert(buffer.end(), frame.begin(), frame.end());
  }

  rf_joy::ibus::ScanResult result;
  const auto decoded = collect(buffer, result);

  ASSERT_EQ(decoded.size(), 3U);
  EXPECT_EQ(decoded[0][0], 1111);
  EXPECT_EQ(decoded[1][0], 1222);
  EXPECT_EQ(decoded[2][0], 1333);
  EXPECT_EQ(result.consumed, 3 * kPacketSize);
  EXPECT_EQ(result.discarded_bytes, 0U);
}

TEST(IbusScan, ResynchronisesAfterLeadingGarbage)
{
  std::vector<std::uint8_t> buffer{0x00, 0xFF, 0x20, 0x13, 0x40, 0x20, 0x20};
  const auto garbage_size = buffer.size();
  const auto frame = make_frame({1444});
  buffer.insert(buffer.end(), frame.begin(), frame.end());

  rf_joy::ibus::ScanResult result;
  const auto decoded = collect(buffer, result);

  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0][0], 1444);
  EXPECT_EQ(result.discarded_bytes, garbage_size);
  EXPECT_EQ(result.consumed, buffer.size());
}

// The regression this guards: 0x20 is a plausible payload byte, so a scan that only looks
// for the length byte locks onto the wrong offset. Here the first frame's header is broken
// and its payload contains a literal 0x20 0x40 pair, so the scan has to walk past both and
// still land on the intact frame that follows.
TEST(IbusScan, IgnoresHeaderLookalikeInsidePayload)
{
  std::vector<std::uint16_t> raw(kChannelCount, 1500);
  raw[3] = 0x4020;  // little endian bytes 0x20 0x40
  auto decoy = make_frame(raw);
  decoy[0] = 0x00;  // break the real header so the scan must slide through the payload
  decoy[1] = 0x00;

  const auto good = make_frame({1777});
  std::vector<std::uint8_t> buffer = decoy;
  buffer.insert(buffer.end(), good.begin(), good.end());

  rf_joy::ibus::ScanResult result;
  const auto decoded = collect(buffer, result);

  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0][0], 1777);
  EXPECT_EQ(result.discarded_bytes, kPacketSize);
  EXPECT_EQ(result.consumed, buffer.size());
}

TEST(IbusScan, LeavesTrailingPartialFrameForTheNextRead)
{
  const auto complete = make_frame({1555});
  const auto partial = make_frame({1666});

  std::vector<std::uint8_t> buffer = complete;
  buffer.insert(buffer.end(), partial.begin(), partial.begin() + 20);

  rf_joy::ibus::ScanResult result;
  const auto decoded = collect(buffer, result);

  ASSERT_EQ(decoded.size(), 1U);
  EXPECT_EQ(decoded[0][0], 1555);
  // Only the complete frame is consumed, so the 20 carried over bytes can be completed by
  // the following read.
  EXPECT_EQ(result.consumed, kPacketSize);
  EXPECT_EQ(buffer.size() - result.consumed, 20U);
}

TEST(IbusScan, ConsumesNothingBelowOneFrame)
{
  const auto frame = make_frame({1888});
  const std::vector<std::uint8_t> buffer(frame.begin(), frame.begin() + kPacketSize - 1);

  rf_joy::ibus::ScanResult result;
  const auto decoded = collect(buffer, result);

  EXPECT_TRUE(decoded.empty());
  EXPECT_EQ(result.consumed, 0U);
  EXPECT_EQ(result.discarded_bytes, 0U);
}

// A wrong baud rate produces a stream that never validates. The carry-over has to stay
// bounded or the buffer grows without limit.
TEST(IbusScan, BoundsCarryOverOnUnparseableStream)
{
  std::vector<std::uint8_t> buffer;
  for (std::size_t i = 0; i < 4096; ++i) {
    buffer.push_back(static_cast<std::uint8_t>((i % 2 == 0) ? kLengthByte : kCommandByte));
  }

  rf_joy::ibus::ScanResult result;
  const auto decoded = collect(buffer, result);

  EXPECT_TRUE(decoded.empty());
  EXPECT_LT(buffer.size() - result.consumed, kPacketSize);
  EXPECT_EQ(result.discarded_bytes, result.consumed);
}
