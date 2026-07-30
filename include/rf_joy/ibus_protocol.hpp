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

#ifndef RF_JOY__IBUS_PROTOCOL_HPP_
#define RF_JOY__IBUS_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rf_joy
{
namespace ibus
{

// FlySky i-BUS servo frame layout:
//   [0]      0x20  frame length in bytes, including this byte and the checksum
//   [1]      0x40  command id for servo/channel data
//   [2..29]  14 channels, little endian uint16 each
//   [30,31]  checksum, little endian uint16, 0xFFFF minus the sum of bytes [0..29]
constexpr std::size_t kPacketSize = 32;
constexpr std::size_t kChannelCount = 14;
constexpr std::uint8_t kLengthByte = 0x20;
constexpr std::uint8_t kCommandByte = 0x40;

// Channel values live in the low 12 bits. With the transmitter in 18 channel mode the
// upper nibbles of the later channels carry the extra channels, so they have to be masked
// off or channels 7..14 read far outside the 1000..2000 range.
constexpr std::uint16_t kChannelMask = 0x0FFF;

/// Checks the trailing checksum of a kPacketSize byte frame.
inline bool verify_checksum(const std::uint8_t * frame)
{
  std::uint16_t checksum = 0xFFFF;
  for (std::size_t i = 0; i < kPacketSize - 2; ++i) {
    checksum = static_cast<std::uint16_t>(checksum - frame[i]);
  }

  const auto received_checksum = static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(frame[kPacketSize - 2]) |
    static_cast<std::uint16_t>(frame[kPacketSize - 1] << 8));

  return checksum == received_checksum;
}

/// Unpacks the kChannelCount channel values of a validated frame.
inline void decode_channels(const std::uint8_t * frame, std::vector<std::uint16_t> & channels)
{
  channels.resize(kChannelCount);
  for (std::size_t i = 0; i < kChannelCount; ++i) {
    const auto base_index = 2 + (i * 2);
    const auto low = static_cast<std::uint16_t>(frame[base_index]);
    const auto high = static_cast<std::uint16_t>(frame[base_index + 1]);
    channels[i] = static_cast<std::uint16_t>((low | (high << 8)) & kChannelMask);
  }
}

struct ScanResult
{
  /// Bytes that may be dropped from the front of the buffer. Whatever follows is either an
  /// incomplete frame or has not been ruled out yet, so it has to be kept for the next read.
  std::size_t consumed{0};
  std::size_t valid_frames{0};
  std::size_t discarded_bytes{0};
};

/// Pulls every complete frame out of a byte stream, calling on_frame(const std::uint8_t *)
/// for each one in arrival order.
///
/// 0x20 also occurs inside the payload, so the command byte and the checksum are what
/// actually establish a frame boundary. On a mismatch the scan slides a single byte rather
/// than discarding a whole packet worth of data, which is what lets it resynchronise after
/// starting mid-frame or losing bytes. Because unresolved bytes stay behind, the caller's
/// buffer keeps at most kPacketSize - 1 bytes of carry-over even when no frame ever
/// validates, for instance under a wrong baud rate.
template<typename FrameHandler>
ScanResult scan_frames(const std::vector<std::uint8_t> & buffer, FrameHandler && on_frame)
{
  ScanResult result;

  while (result.consumed + kPacketSize <= buffer.size()) {
    const std::uint8_t * frame = buffer.data() + result.consumed;

    if (frame[0] != kLengthByte || frame[1] != kCommandByte || !verify_checksum(frame)) {
      ++result.consumed;
      ++result.discarded_bytes;
      continue;
    }

    on_frame(frame);
    ++result.valid_frames;
    result.consumed += kPacketSize;
  }

  return result;
}

}  // namespace ibus
}  // namespace rf_joy

#endif  // RF_JOY__IBUS_PROTOCOL_HPP_
