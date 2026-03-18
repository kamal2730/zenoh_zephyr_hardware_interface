#ifndef ZENOH_ZEPHYR_HARDWARE_INTERFACE__ZENOH_ZEPHYR_HARDWARE_HPP_
#define ZENOH_ZEPHYR_HARDWARE_INTERFACE__ZENOH_ZEPHYR_HARDWARE_HPP_

#include <memory>
#include <string>
#include <mutex>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <zenoh.hxx>

namespace zenoh_zephyr_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ZenohZephyrHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ZenohZephyrHardware)
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  private:
  // Zenoh communication
  std::unique_ptr<zenoh::Session> zenoh_session_;
  std::unique_ptr<zenoh::Subscriber<void>> zenoh_subscriber_;
  std::optional<zenoh::Publisher> zenoh_publisher_;
  std::string zenoh_endpoint_;
  std::string zenoh_mode_;
  std::string sub_topic_name_;
  std::string pub_topic_name_;

  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  mutable std::mutex data_mutex_;
  std::vector<double> hw_state_buffer_;
};

}  // namespace zenoh_zephyr_control

#endif  // ZENOH_ZEPHYR_HARDWARE_INTERFACE__ZENOH_ZEPHYR_HARDWARE_HPP_