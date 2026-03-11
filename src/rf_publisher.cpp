#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include <libserial/SerialPort.h>
#include <libserial/SerialPortConstants.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

namespace
{
constexpr std::size_t kIbusPacketSize = 32;
constexpr std::size_t kIbusChannelCount = 14;
constexpr std::uint8_t kIbusHeader = 0x20;
constexpr int kDefaultReadTimeoutMs = 50;
constexpr int kDefaultBaudRate = 115200;
constexpr double kDefaultPublishRateHz = 30.0;

bool verify_checksum(const std::vector<std::uint8_t> &data)
{
  if (data.size() != kIbusPacketSize) {
    return false;
  }

  std::uint16_t checksum = 0xFFFF;
  for (std::size_t i = 0; i < kIbusPacketSize - 2; ++i) {
    checksum -= data[i];
  }

  const auto received_checksum = static_cast<std::uint16_t>(data[kIbusPacketSize - 2]) |
    (static_cast<std::uint16_t>(data[kIbusPacketSize - 1]) << 8);

  return checksum == received_checksum;
}

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
}  // namespace

class RfPublisherNode : public rclcpp::Node
{
public:
  RfPublisherNode()
  : Node("rf_publisher_node")
  {
    serial_port_name_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyTHS1");
    read_timeout_ms_ = this->declare_parameter<int>("read_timeout_ms", kDefaultReadTimeoutMs);
    const auto baud_rate = this->declare_parameter<int>("baud_rate", kDefaultBaudRate);
    const auto publish_rate_hz =
      this->declare_parameter<double>("publish_rate_hz", kDefaultPublishRateHz);

    if (read_timeout_ms_ <= 0) {
      throw std::invalid_argument("read_timeout_ms must be greater than 0");
    }
    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be greater than 0");
    }

    publisher_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("/rf", 10);

    open_serial_port(baud_rate);

    const auto timer_period =
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / publish_rate_hz));
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&RfPublisherNode::publish_frame, this));
  }

  ~RfPublisherNode() override
  {
    if (serial_port_.IsOpen()) {
      serial_port_.Close();
    }
  }

private:
  void open_serial_port(const int baud_rate)
  {
    serial_port_.Open(serial_port_name_);
    serial_port_.SetBaudRate(to_baud_rate(baud_rate));
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_port_.SetSerialPortBlockingStatus(true);

    if (!serial_port_.IsOpen()) {
      throw LibSerial::OpenFailed("Port open failed after configuration");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "IBUS receiver on %s (%d-8N1)",
      serial_port_name_.c_str(),
      baud_rate);
  }

  bool read_packet(std::vector<std::uint8_t> &packet)
  {
    packet.clear();
    packet.reserve(kIbusPacketSize);

    while (rclcpp::ok()) {
      std::uint8_t byte = 0;
      try {
        serial_port_.ReadByte(byte, read_timeout_ms_);
      } catch (const LibSerial::ReadTimeout &) {
        return false;
      }

      if (byte != kIbusHeader) {
        continue;
      }

      packet.push_back(byte);

      for (std::size_t i = 1; i < kIbusPacketSize; ++i) {
        try {
          serial_port_.ReadByte(byte, read_timeout_ms_);
        } catch (const LibSerial::ReadTimeout &) {
          packet.clear();
          return false;
        }
        packet.push_back(byte);
      }

      return true;
    }

    return false;
  }

  bool read_channels(std::vector<std::uint16_t> &channels)
  {
    std::vector<std::uint8_t> packet;
    if (!read_packet(packet) || !verify_checksum(packet)) {
      return false;
    }

    channels.clear();
    channels.reserve(kIbusChannelCount);
    for (std::size_t i = 0; i < kIbusChannelCount; ++i) {
      const auto base_index = 2 + (i * 2);
      const auto channel_value = static_cast<std::uint16_t>(packet[base_index]) |
        (static_cast<std::uint16_t>(packet[base_index + 1]) << 8);
      channels.push_back(channel_value);
    }

    return true;
  }

  void publish_frame()
  {
    std::vector<std::uint16_t> channels;
    if (!read_channels(channels)) {
      return;
    }

    last_channels_ = channels;

    std_msgs::msg::UInt16MultiArray message;
    message.data = last_channels_;
    publisher_->publish(message);
  }

  std::string serial_port_name_;
  int read_timeout_ms_{kDefaultReadTimeoutMs};
  LibSerial::SerialPort serial_port_;
  std::vector<std::uint16_t> last_channels_ =
    std::vector<std::uint16_t>(kIbusChannelCount, 1500);
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<RfPublisherNode>());
  } catch (const LibSerial::OpenFailed &error) {
    RCLCPP_ERROR(rclcpp::get_logger("rf_publisher_node"), "Serial open failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("rf_publisher_node"), "Unhandled exception: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
