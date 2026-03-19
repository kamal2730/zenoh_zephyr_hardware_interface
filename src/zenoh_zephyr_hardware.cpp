#include "zenoh_zephyr_hardware_interface/zenoh_zephyr_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zenoh_zephyr_hardware_interface/msg.h"

namespace zenoh_zephyr_control
{

CallbackReturn ZenohZephyrHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), 90);
  hw_commands_.resize(info_.joints.size(),90);

  hw_state_buffer_[0].resize(info_.joints.size(), 0.0);
  hw_state_buffer_[1].resize(info_.joints.size(), 0.0);

  try {
    zenoh_endpoint_ = info_.hardware_parameters.at("zenoh_endpoint");
    sub_topic_name_ = info_.hardware_parameters.at("joint_history_topic");
    pub_topic_name_ = info_.hardware_parameters.at("joint_command_topic");
    zenoh_mode_ = info_.hardware_parameters.count("zenoh_mode") ? 
                  info_.hardware_parameters.at("zenoh_mode") : "client";
  } catch (const std::out_of_range & e) {
    RCLCPP_FATAL(rclcpp::get_logger("ZenohZephyrHardware"), "Missing required parameter in URDF!");
    return CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ZenohZephyrHardware"),
        "Joint '%s' has %zu command interfaces. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ZenohZephyrHardware"),
        "Joint '%s' has %zu state interfaces. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ZenohZephyrHardware"), "Hardware Interface Initialized!");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ZenohZephyrHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ZenohZephyrHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

CallbackReturn ZenohZephyrHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  hw_state_buffer_[0].resize(info_.joints.size(), 0.0);
  hw_state_buffer_[1].resize(info_.joints.size(), 0.0);

  auto config = zenoh::Config::create_default();
  config.insert_json5("connect/endpoints", "[\"" + zenoh_endpoint_ + "\"]");
  config.insert_json5("mode", "\"" + zenoh_mode_ + "\"");
  try {
    zenoh_session_ = std::make_unique<zenoh::Session>(zenoh::Session::open(std::move(config)));
    zenoh_publisher_=zenoh_session_->declare_publisher(pub_topic_name_);
    zenoh_subscriber_ = std::make_unique<zenoh::Subscriber<void>>(
      zenoh_session_->declare_subscriber(
        sub_topic_name_, 
        [this](const zenoh::Sample& sample) {
            const auto& payload = sample.get_payload();
            size_t num_joints = hw_state_buffer_[0].size();
            size_t expected_size = sizeof(state_msg_t) * num_joints;

            if (payload.size() == expected_size) {
                int write_idx = 1 - write_index_.load(std::memory_order_acquire);
                std::vector<uint8_t> vec = payload.as_vector();
                const auto* incoming = reinterpret_cast<const state_msg_t*>(vec.data());
                for (size_t i = 0; i < num_joints; ++i) {
                    hw_state_buffer_[write_idx][i] = static_cast<double>(incoming[i].position);
                }
                write_index_.store(write_idx, std::memory_order_release);
                new_data_available_.store(true, std::memory_order_release);
               
            }
        },
        []() {} 
      )
    );
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("ZenohZephyrHardware"), "Failed to start Zenoh: %s", e.what());
    return CallbackReturn::ERROR;
  }
  // Set default values for joints
  for (size_t i = 0; i < hw_states_.size(); i++)
  {
    if (std::isnan(hw_states_[i]))
    {
      hw_states_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ZenohZephyrHardware"), "Hardware activated! Ready for control loop.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ZenohZephyrHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  zenoh_subscriber_.reset();  
  if (zenoh_publisher_.has_value()) zenoh_publisher_.reset();
  zenoh_session_.reset();
  RCLCPP_INFO(rclcpp::get_logger("ZenohZephyrHardware"), "Hardware deactivated.");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ZenohZephyrHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
    int idx = write_index_.load(std::memory_order_acquire);
    for (size_t i = 0; i < hw_states_.size(); i++) {
        hw_states_[i] = hw_state_buffer_[idx][i];
    }
    new_data_available_.store(false, std::memory_order_relaxed);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZenohZephyrHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
    if (!zenoh_publisher_.has_value()) return hardware_interface::return_type::OK;

    std::vector<command_msg_t> commands_to_send(hw_commands_.size());
    for (size_t i = 0; i < hw_commands_.size(); i++) {
        commands_to_send[i].command = static_cast<float>(hw_commands_[i]);
    }
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(commands_to_send.data());
    std::vector<uint8_t> data(ptr, ptr + commands_to_send.size() * sizeof(command_msg_t));
    zenoh_publisher_->put(zenoh::Bytes(std::move(data)));
    return hardware_interface::return_type::OK;
}

}  // namespace zenoh_zephyr_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  zenoh_zephyr_control::ZenohZephyrHardware, hardware_interface::SystemInterface)