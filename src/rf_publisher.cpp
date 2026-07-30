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

#include <poll.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

#include "rf_joy/ibus_protocol.hpp"

namespace
{
// Upper bound for a single bulk read so a full kernel buffer cannot force one large
// allocation. Any remainder is picked up on the next iteration without waiting.
constexpr std::size_t kMaxReadChunkBytes = 1024;

constexpr int kDefaultBaudRate = 115200;
constexpr int kDefaultReadTimeoutMs = 50;

// The port is drained at the full i-BUS rate regardless, so this only bounds how often the
// freshest frame is handed to DDS. Publishing every frame at about 140 Hz multiplies the
// publish, take and callback work by roughly five for no gain in control quality, so the
// default keeps the previous 30 Hz topic load.
constexpr double kDefaultMaxPublishRateHz = 30.0;
constexpr int kDefaultSignalTimeoutMs = 500;
constexpr int kDefaultReconnectIntervalMs = 1000;
constexpr int kDefaultQosDepth = 10;
constexpr int kShutdownPollIntervalMs = 20;

LibSerial::BaudRate to_baud_rate(const int baud_rate)
{
  switch (baud_rate) {
    case 1200:
      return LibSerial::BaudRate::BAUD_1200;
    case 2400:
      return LibSerial::BaudRate::BAUD_2400;
    case 4800:
      return LibSerial::BaudRate::BAUD_4800;
    case 9600:
      return LibSerial::BaudRate::BAUD_9600;
    case 19200:
      return LibSerial::BaudRate::BAUD_19200;
    case 38400:
      return LibSerial::BaudRate::BAUD_38400;
    case 57600:
      return LibSerial::BaudRate::BAUD_57600;
    case 115200:
      return LibSerial::BaudRate::BAUD_115200;
    case 230400:
      return LibSerial::BaudRate::BAUD_230400;
    default:
      throw std::invalid_argument("Unsupported baud rate: " + std::to_string(baud_rate));
  }
}

rclcpp::QoS make_qos(const std::string & reliability, const int depth)
{
  if (depth <= 0) {
    throw std::invalid_argument("QoS depth must be greater than 0");
  }

  rclcpp::QoS qos(rclcpp::KeepLast(static_cast<std::size_t>(depth)));
  if (reliability == "reliable") {
    qos.reliable();
  } else if (reliability == "best_effort") {
    qos.best_effort();
  } else {
    throw std::invalid_argument(
      "QoS reliability must be \"reliable\" or \"best_effort\", got: " + reliability);
  }

  return qos;
}
}  // namespace

// Reads i-BUS frames from a serial port and republishes the channel values on /rf.
//
// The port is served by a dedicated thread rather than a timer callback. A timer that
// consumes one frame per tick falls behind whenever the tick period is longer than the
// i-BUS frame period (about 7 ms), and the backlog ends up as a fixed latency once the
// kernel buffer saturates. The reader thread instead takes every byte the port has and
// keeps only the newest complete frame, so latency stays bounded by the frame rate.
class RfPublisherNode : public rclcpp::Node
{
public:
  RfPublisherNode()
  : Node("rf_publisher_node")
  {
    serial_port_name_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyTHS1");
    baud_rate_ = this->declare_parameter<int>("baud_rate", kDefaultBaudRate);
    read_timeout_ms_ = this->declare_parameter<int>("read_timeout_ms", kDefaultReadTimeoutMs);
    publish_latest_only_ = this->declare_parameter<bool>("publish_latest_only", true);
    signal_timeout_ms_ =
      this->declare_parameter<int>("signal_timeout_ms", kDefaultSignalTimeoutMs);
    reconnect_interval_ms_ =
      this->declare_parameter<int>("reconnect_interval_ms", kDefaultReconnectIntervalMs);
    const auto max_publish_rate_hz =
      this->declare_parameter<double>("max_publish_rate_hz", kDefaultMaxPublishRateHz);
    const auto qos_reliability =
      this->declare_parameter<std::string>("qos.reliability", "reliable");
    const auto qos_depth = this->declare_parameter<int>("qos.depth", kDefaultQosDepth);

    if (read_timeout_ms_ <= 0) {
      throw std::invalid_argument("read_timeout_ms must be greater than 0");
    }
    if (signal_timeout_ms_ <= 0) {
      throw std::invalid_argument("signal_timeout_ms must be greater than 0");
    }
    if (reconnect_interval_ms_ <= 0) {
      throw std::invalid_argument("reconnect_interval_ms must be greater than 0");
    }
    if (max_publish_rate_hz < 0.0) {
      throw std::invalid_argument("max_publish_rate_hz must be zero or greater");
    }

    // Reject an unsupported baud rate here instead of inside the reader thread, where it
    // would be reported once per reconnect attempt forever.
    to_baud_rate(baud_rate_);

    if (max_publish_rate_hz > 0.0) {
      max_publish_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / max_publish_rate_hz));
    }

    publisher_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>(
      "/rf", make_qos(qos_reliability, qos_depth));

    rx_buffer_.reserve(kMaxReadChunkBytes + rf_joy::ibus::kPacketSize);

    const auto start_time = std::chrono::steady_clock::now();
    last_frame_time_ = start_time;
    last_publish_time_ = start_time - max_publish_period_;

    running_ = true;
    reader_thread_ = std::thread(&RfPublisherNode::read_loop, this);
  }

  ~RfPublisherNode() override
  {
    running_ = false;
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
    close_serial_port();
  }

private:
  bool try_open_serial_port()
  {
    try {
      serial_port_.Open(serial_port_name_);
      serial_port_.SetBaudRate(to_baud_rate(baud_rate_));
      serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
      serial_port_.SetSerialPortBlockingStatus(true);

      if (!serial_port_.IsOpen()) {
        throw LibSerial::OpenFailed("Port open failed after configuration");
      }
    } catch (const std::exception & error) {
      if (!open_failure_reported_) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Cannot open %s: %s. Retrying every %d ms.",
          serial_port_name_.c_str(),
          error.what(),
          reconnect_interval_ms_);
        open_failure_reported_ = true;
      }
      close_serial_port();
      return false;
    }

    // Stale bytes from a previous connection would only feed the resync scan.
    rx_buffer_.clear();
    open_failure_reported_ = false;

    RCLCPP_INFO(
      this->get_logger(),
      "IBUS receiver on %s (%d-8N1)",
      serial_port_name_.c_str(),
      baud_rate_);

    return true;
  }

  void close_serial_port()
  {
    try {
      if (serial_port_.IsOpen()) {
        serial_port_.Close();
      }
    } catch (const std::exception &) {
      // Nothing useful is left to do with a port that will not close.
    }
  }

  void read_loop()
  {
    while (running_ && rclcpp::ok()) {
      if (!serial_port_.IsOpen()) {
        if (!try_open_serial_port()) {
          sleep_ms(reconnect_interval_ms_);
        }
        continue;
      }

      try {
        if (read_available_bytes()) {
          parse_and_publish();
        }
      } catch (const std::exception & error) {
        // Losing the device mid-run used to escape spin() and kill the process. Drop the
        // handle instead and let the reconnect path pick the device back up.
        RCLCPP_ERROR(
          this->get_logger(),
          "Serial read on %s failed: %s. Reopening the port.",
          serial_port_name_.c_str(),
          error.what());
        close_serial_port();
        sleep_ms(reconnect_interval_ms_);
        continue;
      }

      report_signal_state();
    }
  }

  // Returns false when the wait timed out without producing any data.
  //
  // This reads the descriptor directly instead of going through LibSerial's Read(), because
  // Read(buffer, n, msTimeout) cannot be used safely on a blocking port: it waits inside
  // read() for all n bytes and only checks its own timeout between iterations, so it never
  // times out. Sizing n from GetNumberOfBytesAvailable() therefore stalls indefinitely
  // whenever fewer bytes are delivered than the queue advertised, which is what dragged the
  // publish rate down on real UART hardware. poll() plus one read() of whatever is queued
  // has neither problem: it parks the thread while the receiver is quiet, hands over the
  // whole backlog in a single call, and never waits on a byte count that may not arrive.
  bool read_available_bytes()
  {
    const int fd = serial_port_.GetFileDescriptor();

    struct pollfd poll_descriptor {};
    poll_descriptor.fd = fd;
    poll_descriptor.events = POLLIN;

    const int ready = ::poll(&poll_descriptor, 1, read_timeout_ms_);
    if (ready == 0) {
      return false;
    }
    if (ready < 0) {
      if (errno == EINTR) {
        return false;
      }
      throw std::runtime_error(std::string("poll failed: ") + std::strerror(errno));
    }

    std::array<std::uint8_t, kMaxReadChunkBytes> chunk{};
    const auto received = ::read(fd, chunk.data(), chunk.size());
    if (received < 0) {
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        return false;
      }
      throw std::runtime_error(std::string("read failed: ") + std::strerror(errno));
    }
    if (received == 0) {
      return false;
    }

    rx_buffer_.insert(
      rx_buffer_.end(),
      chunk.begin(),
      chunk.begin() + static_cast<std::ptrdiff_t>(received));

    return true;
  }

  void parse_and_publish()
  {
    const auto scan = rf_joy::ibus::scan_frames(
      rx_buffer_,
      [this](const std::uint8_t * frame) {
        rf_joy::ibus::decode_channels(frame, latest_channels_);
        if (!publish_latest_only_) {
          publish_channels();
        }
      });

    rx_buffer_.erase(
      rx_buffer_.begin(),
      rx_buffer_.begin() + static_cast<std::ptrdiff_t>(scan.consumed));
    dropped_bytes_ += scan.discarded_bytes;

    if (scan.valid_frames == 0) {
      return;
    }

    frame_count_ += scan.valid_frames;
    last_frame_time_ = std::chrono::steady_clock::now();

    if (signal_lost_) {
      RCLCPP_INFO(
        this->get_logger(),
        "IBUS frames received again on %s",
        serial_port_name_.c_str());
      signal_lost_ = false;
    }

    if (publish_latest_only_) {
      publish_channels();
    }

    RCLCPP_DEBUG(
      this->get_logger(),
      "IBUS link: %zu frames, %zu bytes discarded during resync",
      frame_count_,
      dropped_bytes_);
  }

  void publish_channels()
  {
    if (max_publish_period_ > std::chrono::steady_clock::duration::zero()) {
      const auto current_time = std::chrono::steady_clock::now();
      if (current_time - last_publish_time_ < max_publish_period_) {
        return;
      }
      last_publish_time_ = current_time;
    }

    std_msgs::msg::UInt16MultiArray message;
    message.data = latest_channels_;
    publisher_->publish(message);
  }

  void report_signal_state()
  {
    if (signal_lost_) {
      return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - last_frame_time_;
    if (elapsed < std::chrono::milliseconds(signal_timeout_ms_)) {
      return;
    }

    RCLCPP_WARN(
      this->get_logger(),
      "No valid IBUS frame on %s for %d ms. Check the receiver power, the i-BUS output "
      "mode and the baud rate.",
      serial_port_name_.c_str(),
      signal_timeout_ms_);
    signal_lost_ = true;
  }

  // Sleeps in short steps so a pending shutdown is not delayed by a full interval.
  void sleep_ms(const int duration_ms)
  {
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_ms);
    while (running_ && rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(kShutdownPollIntervalMs));
    }
  }

  std::string serial_port_name_;
  int baud_rate_{kDefaultBaudRate};
  int read_timeout_ms_{kDefaultReadTimeoutMs};
  int signal_timeout_ms_{kDefaultSignalTimeoutMs};
  int reconnect_interval_ms_{kDefaultReconnectIntervalMs};
  bool publish_latest_only_{true};
  std::chrono::steady_clock::duration max_publish_period_{
    std::chrono::steady_clock::duration::zero()};

  LibSerial::SerialPort serial_port_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr publisher_;

  // Touched only by the reader thread once it has been started.
  std::vector<std::uint8_t> rx_buffer_;
  std::vector<std::uint16_t> latest_channels_;
  std::chrono::steady_clock::time_point last_frame_time_;
  std::chrono::steady_clock::time_point last_publish_time_;
  std::size_t frame_count_{0};
  std::size_t dropped_bytes_{0};
  bool signal_lost_{false};
  bool open_failure_reported_{false};

  std::atomic<bool> running_{false};
  std::thread reader_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<RfPublisherNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("rf_publisher_node"), "Unhandled exception: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
