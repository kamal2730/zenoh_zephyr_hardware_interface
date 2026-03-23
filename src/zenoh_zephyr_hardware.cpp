#include "zenoh_zephyr_hardware_interface/zenoh_zephyr_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zenoh_zephyr_hardware_interface/msg.h"

uint64_t get_time_us() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();
}

namespace zenoh_zephyr_control
{

CallbackReturn ZenohZephyrHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  size_t n = info_.joints.size();

  hw_commands_.resize(n, 0.0);

  hw_states_raw_.resize(n);
  hw_pos_.resize(n, 0.0);
  hw_vel_.resize(n, 0.0);
  hw_eff_.resize(n, 0.0);

  hw_state_buffer_[0].resize(n);
  hw_state_buffer_[1].resize(n);

  try {
    zenoh_endpoint_ = info_.hardware_parameters.at("zenoh_endpoint");
    sub_topic_name_ = info_.hardware_parameters.at("joint_history_topic");
    pub_topic_name_ = info_.hardware_parameters.at("joint_command_topic");
    zenoh_mode_ = info_.hardware_parameters.count("zenoh_mode") ?
                  info_.hardware_parameters.at("zenoh_mode") : "client";
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("ZenohZephyrHardware"), "Missing required parameter!");
    return CallbackReturn::ERROR;
  }

  // NOW expect 3 states
  for (const auto & joint : info_.joints)
  {
    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(rclcpp::get_logger("ZenohZephyrHardware"),
        "Joint '%s' must have 3 state interfaces!", joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ZenohZephyrHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    si.emplace_back(info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION,
                    &hw_pos_[i]);

    si.emplace_back(info_.joints[i].name,
                    hardware_interface::HW_IF_VELOCITY,
                    &hw_vel_[i]);

    si.emplace_back(info_.joints[i].name,
                    hardware_interface::HW_IF_EFFORT,
                    &hw_eff_[i]);
  }

  return si;
}

std::vector<hardware_interface::CommandInterface>
ZenohZephyrHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    ci.emplace_back(info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION,
                    &hw_commands_[i]);
  }

  return ci;
}

CallbackReturn ZenohZephyrHardware::on_activate(const rclcpp_lifecycle::State &)
{
  auto node = rclcpp::Node::make_shared("zenoh_hw_latency");
  latency_pub_ = node->create_publisher<std_msgs::msg::Float64>(
    "latency", rclcpp::SensorDataQoS());

  auto config = zenoh::Config::create_default();
  config.insert_json5("connect/endpoints", "[\"" + zenoh_endpoint_ + "\"]");
  config.insert_json5("mode", "\"" + zenoh_mode_ + "\"");

  try {
    zenoh_session_ = std::make_unique<zenoh::Session>(
      zenoh::Session::open(std::move(config)));

    zenoh_publisher_ = zenoh_session_->declare_publisher(pub_topic_name_);

    zenoh_subscriber_ = std::make_unique<zenoh::Subscriber<void>>(
      zenoh_session_->declare_subscriber(
        sub_topic_name_,
        [this](const zenoh::Sample& sample) {

          const auto& payload = sample.get_payload();
          size_t n = hw_states_raw_.size();
          size_t expected = sizeof(state_msg_t) * n;

          if (payload.size() != expected) return;

          int write_idx = 1 - write_index_.load(std::memory_order_acquire);

          std::vector<uint8_t> vec = payload.as_vector();
          const auto* incoming =
            reinterpret_cast<const state_msg_t*>(vec.data());

          for (size_t i = 0; i < n; ++i) {
            hw_state_buffer_[write_idx][i] = incoming[i];

            uint64_t now = get_time_us();
            uint64_t latency = now - incoming[i].timestamp;

            std_msgs::msg::Float64 msg;
            msg.data = latency / 1000.0;
            latency_pub_->publish(msg);
          }

          write_index_.store(write_idx, std::memory_order_release);
          new_data_available_.store(true, std::memory_order_release);
        },
        []() {}
      )
    );

  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("ZenohZephyrHardware"),
                 "Zenoh failed: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ZenohZephyrHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  zenoh_subscriber_.reset();
  if (zenoh_publisher_) zenoh_publisher_.reset();
  zenoh_session_.reset();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ZenohZephyrHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  int idx = write_index_.load(std::memory_order_acquire);

  for (size_t i = 0; i < hw_states_raw_.size(); i++)
  {
    hw_states_raw_[i] = hw_state_buffer_[idx][i];

    hw_pos_[i] = hw_states_raw_[i].position;
    hw_vel_[i] = hw_states_raw_[i].velocity;
    hw_eff_[i] = hw_states_raw_[i].effort;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZenohZephyrHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!zenoh_publisher_) return hardware_interface::return_type::OK;

  std::vector<command_msg_t> commands(hw_commands_.size());

  for (size_t i = 0; i < hw_commands_.size(); i++) {
    commands[i].timestamp = get_time_us();
    commands[i].command = static_cast<float>(hw_commands_[i]);
  }

  const uint8_t* ptr = reinterpret_cast<const uint8_t*>(commands.data());
  std::vector<uint8_t> data(ptr, ptr + commands.size() * sizeof(command_msg_t));

  zenoh_publisher_->put(zenoh::Bytes(std::move(data)));

  return hardware_interface::return_type::OK;
}

}  // namespace zenoh_zephyr_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  zenoh_zephyr_control::ZenohZephyrHardware,
  hardware_interface::SystemInterface)