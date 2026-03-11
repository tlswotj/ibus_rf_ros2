#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

namespace
{
bool evaluate_threshold(const std::uint16_t value, const int threshold, const bool active_when_above)
{
  return active_when_above ? value > threshold : value < threshold;
}

float normalize_axis(
  const std::uint16_t value,
  const double offset,
  const double scale,
  const bool invert)
{
  if (std::abs(scale) < std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("Axis scale must not be zero");
  }

  double normalized = (static_cast<double>(value) - offset) / scale;
  if (invert) {
    normalized *= -1.0;
  }

  normalized = std::max(-1.0, std::min(1.0, normalized));
  return static_cast<float>(normalized);
}
}  // namespace

class JoyPublisher : public rclcpp::Node
{
public:
  JoyPublisher()
  : Node("rf_to_joy_node")
  {
    kill_switch_channel_ = this->declare_parameter<int>("kill_switch.channel", 4);
    kill_switch_threshold_ = this->declare_parameter<int>("kill_switch.threshold", 1600);
    kill_switch_active_when_above_ =
      this->declare_parameter<bool>("kill_switch.active_when_above", true);

    publish_gate_enabled_ = this->declare_parameter<bool>("publish_gate.enabled", true);
    publish_gate_channel_ = this->declare_parameter<int>("publish_gate.channel", 5);
    publish_gate_threshold_ = this->declare_parameter<int>("publish_gate.threshold", 1600);
    publish_gate_active_when_above_ =
      this->declare_parameter<bool>("publish_gate.active_when_above", false);

    axis_channels_ = this->declare_parameter<std::vector<int64_t>>(
      "axes.channels", std::vector<int64_t>{0, 1, 2, 3});
    axis_offsets_ = this->declare_parameter<std::vector<double>>(
      "axes.offsets", std::vector<double>{1500.0, 1500.0, 1500.0, 1500.0});
    axis_scales_ = this->declare_parameter<std::vector<double>>(
      "axes.scales", std::vector<double>{500.0, 500.0, 500.0, 500.0});
    axis_invert_ = this->declare_parameter<std::vector<bool>>(
      "axes.invert", std::vector<bool>{false, false, false, false});

    button_channels_ = this->declare_parameter<std::vector<int64_t>>(
      "buttons.channels", std::vector<int64_t>{5, 5});
    button_thresholds_ = this->declare_parameter<std::vector<int64_t>>(
      "buttons.thresholds", std::vector<int64_t>{1300, 1600});
    button_active_when_above_ = this->declare_parameter<std::vector<bool>>(
      "buttons.active_when_above", std::vector<bool>{true, true});

    validate_parameters();

    joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", 10);
    rf_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/rf",
      10,
      std::bind(&JoyPublisher::rf_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "rf_to_joy node started with %zu axes and %zu buttons",
      axis_channels_.size(), button_channels_.size());
  }

private:
  void validate_parameters()
  {
    const auto axis_count = axis_channels_.size();
    if (axis_offsets_.size() != axis_count ||
      axis_scales_.size() != axis_count ||
      axis_invert_.size() != axis_count)
    {
      throw std::invalid_argument(
              "axes.channels, axes.offsets, axes.scales and axes.invert must have the same length");
    }

    const auto button_count = button_channels_.size();
    if (button_thresholds_.size() != button_count ||
      button_active_when_above_.size() != button_count)
    {
      throw std::invalid_argument(
              "buttons.channels, buttons.thresholds and buttons.active_when_above must have the same length");
    }

    auto validate_channel = [](const int channel, const std::string &name) {
        if (channel < 0) {
          throw std::invalid_argument(name + " must be zero or greater");
        }
      };

    validate_channel(kill_switch_channel_, "kill_switch.channel");
    if (publish_gate_enabled_) {
      validate_channel(publish_gate_channel_, "publish_gate.channel");
    }

    for (std::size_t i = 0; i < axis_count; ++i) {
      validate_channel(static_cast<int>(axis_channels_[i]), "axes.channels");
      if (std::abs(axis_scales_[i]) < std::numeric_limits<double>::epsilon()) {
        throw std::invalid_argument("axes.scales must not contain zero");
      }
    }

    for (const auto channel : button_channels_) {
      validate_channel(static_cast<int>(channel), "buttons.channels");
    }
  }

  void rf_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg)
  {
    if (!channels_available(msg->data)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "RF data size %zu is smaller than required for the configured mapping",
        msg->data.size());
      return;
    }

    if (!kill_switch_active(msg->data)) {
      publish_zero_joy();
      return;
    }

    if (publish_gate_enabled_ && !publish_gate_active(msg->data)) {
      return;
    }

    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = this->now();
    joy_msg.axes.resize(axis_channels_.size(), 0.0F);
    joy_msg.buttons.resize(button_channels_.size(), 0);

    for (std::size_t i = 0; i < axis_channels_.size(); ++i) {
      joy_msg.axes[i] = normalize_axis(
        msg->data[axis_channels_[i]],
        axis_offsets_[i],
        axis_scales_[i],
        axis_invert_[i]);
    }

    for (std::size_t i = 0; i < button_channels_.size(); ++i) {
      joy_msg.buttons[i] = evaluate_threshold(
        msg->data[button_channels_[i]],
        static_cast<int>(button_thresholds_[i]),
        button_active_when_above_[i]) ? 1 : 0;
    }

    joy_publisher_->publish(joy_msg);
  }

  bool channels_available(const std::vector<std::uint16_t> &channels) const
  {
    int max_channel = kill_switch_channel_;
    if (publish_gate_enabled_) {
      max_channel = std::max(max_channel, publish_gate_channel_);
    }

    for (const auto channel : axis_channels_) {
      max_channel = std::max(max_channel, static_cast<int>(channel));
    }
    for (const auto channel : button_channels_) {
      max_channel = std::max(max_channel, static_cast<int>(channel));
    }

    return channels.size() > static_cast<std::size_t>(max_channel);
  }

  bool kill_switch_active(const std::vector<std::uint16_t> &channels) const
  {
    return evaluate_threshold(
      channels[kill_switch_channel_],
      kill_switch_threshold_,
      kill_switch_active_when_above_);
  }

  bool publish_gate_active(const std::vector<std::uint16_t> &channels) const
  {
    return evaluate_threshold(
      channels[publish_gate_channel_],
      publish_gate_threshold_,
      publish_gate_active_when_above_);
  }

  void publish_zero_joy()
  {
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = this->now();
    joy_msg.axes.resize(axis_channels_.size(), 0.0F);
    joy_msg.buttons.resize(button_channels_.size(), 0);
    joy_publisher_->publish(joy_msg);
  }

  int kill_switch_channel_{4};
  int kill_switch_threshold_{1600};
  bool kill_switch_active_when_above_{true};
  bool publish_gate_enabled_{true};
  int publish_gate_channel_{5};
  int publish_gate_threshold_{1600};
  bool publish_gate_active_when_above_{false};
  std::vector<int64_t> axis_channels_;
  std::vector<double> axis_offsets_;
  std::vector<double> axis_scales_;
  std::vector<bool> axis_invert_;
  std::vector<int64_t> button_channels_;
  std::vector<int64_t> button_thresholds_;
  std::vector<bool> button_active_when_above_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr rf_subscriber_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<JoyPublisher>());
  } catch (const std::exception &error) {
    RCLCPP_ERROR(rclcpp::get_logger("rf_to_joy_node"), "Unhandled exception: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
