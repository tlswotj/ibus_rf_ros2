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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

namespace
{
constexpr int kDefaultWatchdogTimeoutMs = 200;
constexpr double kDefaultWatchdogRateHz = 50.0;
constexpr int kDefaultQosDepth = 10;

// rf_publisher_node decodes 14 channels and i-BUS itself tops out at 18, so an index past
// this is a configuration mistake rather than an exotic setup. Rejecting it up front also
// keeps channels_available() from having to compare values that do not survive a narrowing
// cast: a channel of 2^32 silently becomes 0 as an int, passes the size check and then
// indexes the message out of bounds.
constexpr std::int64_t kChannelIndexLimit = 32;

bool evaluate_threshold(
  const std::uint16_t value,
  const int threshold,
  const bool active_when_above)
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

    watchdog_enabled_ = this->declare_parameter<bool>("watchdog.enabled", true);
    watchdog_timeout_ms_ =
      this->declare_parameter<int>("watchdog.timeout_ms", kDefaultWatchdogTimeoutMs);
    const auto watchdog_rate_hz =
      this->declare_parameter<double>("watchdog.rate_hz", kDefaultWatchdogRateHz);
    failsafe_axes_ = this->declare_parameter<std::vector<double>>(
      "watchdog.failsafe_axes", std::vector<double>{});

    const auto rf_reliability =
      this->declare_parameter<std::string>("qos.rf_subscription.reliability", "reliable");
    const auto rf_depth =
      this->declare_parameter<int>("qos.rf_subscription.depth", kDefaultQosDepth);
    const auto joy_reliability =
      this->declare_parameter<std::string>("qos.joy_publisher.reliability", "reliable");
    const auto joy_depth =
      this->declare_parameter<int>("qos.joy_publisher.depth", kDefaultQosDepth);

    validate_parameters(watchdog_rate_hz);

    joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>(
      "/joy", make_qos(joy_reliability, joy_depth));
    rf_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "/rf",
      make_qos(rf_reliability, rf_depth),
      std::bind(&JoyPublisher::rf_callback, this, std::placeholders::_1));

    last_rf_time_ = this->now();

    if (watchdog_enabled_) {
      const auto watchdog_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / watchdog_rate_hz));
      watchdog_timer_ = this->create_wall_timer(
        watchdog_period,
        std::bind(&JoyPublisher::watchdog_callback, this));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "rf_to_joy node started with %zu axes and %zu buttons",
      axis_channels_.size(),
      button_channels_.size());

    if (watchdog_enabled_) {
      RCLCPP_INFO(
        this->get_logger(),
        "RF watchdog active: failsafe Joy is published when /rf is silent for %d ms",
        watchdog_timeout_ms_);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "RF watchdog disabled: /joy simply stops on signal loss and downstream nodes may "
        "keep acting on the last command");
    }
  }

private:
  void validate_parameters(const double watchdog_rate_hz)
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
              "buttons.channels, buttons.thresholds and buttons.active_when_above must have "
              "the same length");
    }

    auto validate_channel = [](const std::int64_t channel, const std::string & name) {
        if (channel < 0 || channel >= kChannelIndexLimit) {
          throw std::invalid_argument(
                  name + " must be within [0, " + std::to_string(kChannelIndexLimit - 1) + "]");
        }
      };

    validate_channel(kill_switch_channel_, "kill_switch.channel");
    if (publish_gate_enabled_) {
      validate_channel(publish_gate_channel_, "publish_gate.channel");
    }

    for (std::size_t i = 0; i < axis_count; ++i) {
      validate_channel(axis_channels_[i], "axes.channels");
      if (std::abs(axis_scales_[i]) < std::numeric_limits<double>::epsilon()) {
        throw std::invalid_argument("axes.scales must not contain zero");
      }
    }

    for (const auto channel : button_channels_) {
      validate_channel(channel, "buttons.channels");
    }

    if (watchdog_enabled_) {
      if (watchdog_timeout_ms_ <= 0) {
        throw std::invalid_argument("watchdog.timeout_ms must be greater than 0");
      }
      if (watchdog_rate_hz <= 0.0) {
        throw std::invalid_argument("watchdog.rate_hz must be greater than 0");
      }
    }

    // An empty list keeps the previous behaviour of centring every axis. It is only a safe
    // default for axes that are actually centred; a throttle mapped over 1000..2000 sits at
    // half output when its axis is 0.0, so those axes need an explicit value here.
    if (failsafe_axes_.empty()) {
      failsafe_axes_.assign(axis_count, 0.0);
    } else if (failsafe_axes_.size() != axis_count) {
      throw std::invalid_argument(
              "watchdog.failsafe_axes must be empty or the same length as axes.channels");
    }

    for (const auto value : failsafe_axes_) {
      if (value < -1.0 || value > 1.0) {
        throw std::invalid_argument("watchdog.failsafe_axes values must be within [-1.0, 1.0]");
      }
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

    // Only frames wide enough to be usable count as a live link. A publisher that keeps
    // sending unusable frames should engage the failsafe, not suppress it.
    last_rf_time_ = this->now();
    rf_received_ = true;
    if (failsafe_engaged_) {
      RCLCPP_INFO(this->get_logger(), "RF link restored, leaving failsafe");
      failsafe_engaged_ = false;
    }

    if (!kill_switch_active(msg->data)) {
      publish_failsafe_joy();
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

  // Runs on the same single threaded executor as rf_callback, so last_rf_time_ needs no
  // synchronisation.
  void watchdog_callback()
  {
    if (rf_received_) {
      const auto elapsed = this->now() - last_rf_time_;
      if (elapsed < rclcpp::Duration(std::chrono::milliseconds(watchdog_timeout_ms_))) {
        return;
      }
    }

    // Before the first frame arrives the failsafe is published as well, so a subscriber
    // never has to distinguish "not started yet" from "link lost".
    if (!failsafe_engaged_) {
      if (rf_received_) {
        RCLCPP_WARN(
          this->get_logger(),
          "No /rf message for %d ms, publishing failsafe Joy",
          watchdog_timeout_ms_);
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "No /rf message received yet, publishing failsafe Joy until the link comes up");
      }
      failsafe_engaged_ = true;
    }

    publish_failsafe_joy();
  }

  bool channels_available(const std::vector<std::uint16_t> & channels) const
  {
    // Every index has been checked against kChannelIndexLimit already, so the widest one
    // is a small non negative number and the cast below is safe.
    std::int64_t max_channel = kill_switch_channel_;
    if (publish_gate_enabled_) {
      max_channel = std::max(max_channel, static_cast<std::int64_t>(publish_gate_channel_));
    }

    for (const auto channel : axis_channels_) {
      max_channel = std::max(max_channel, channel);
    }
    for (const auto channel : button_channels_) {
      max_channel = std::max(max_channel, channel);
    }

    return channels.size() > static_cast<std::size_t>(max_channel);
  }

  bool kill_switch_active(const std::vector<std::uint16_t> & channels) const
  {
    return evaluate_threshold(
      channels[kill_switch_channel_],
      kill_switch_threshold_,
      kill_switch_active_when_above_);
  }

  bool publish_gate_active(const std::vector<std::uint16_t> & channels) const
  {
    return evaluate_threshold(
      channels[publish_gate_channel_],
      publish_gate_threshold_,
      publish_gate_active_when_above_);
  }

  // The safe output state, used both when the kill switch is off and when the watchdog
  // fires. It ignores publish_gate on purpose: a closed gate must not be able to withhold
  // the failsafe.
  void publish_failsafe_joy()
  {
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.stamp = this->now();
    joy_msg.axes.reserve(failsafe_axes_.size());
    for (const auto value : failsafe_axes_) {
      joy_msg.axes.push_back(static_cast<float>(value));
    }
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

  bool watchdog_enabled_{true};
  int watchdog_timeout_ms_{kDefaultWatchdogTimeoutMs};
  std::vector<double> failsafe_axes_;
  rclcpp::Time last_rf_time_;
  bool rf_received_{false};
  bool failsafe_engaged_{false};

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr rf_subscriber_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<JoyPublisher>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("rf_to_joy_node"), "Unhandled exception: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
