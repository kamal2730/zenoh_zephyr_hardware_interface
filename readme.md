# zenoh_zephyr_hardware_interface

ROS 2 hardware interface for communicating with Zephyr RTOS over Zenoh.

## Description

A [ros2_control](https://control.ros.org/) hardware interface plugin that uses [Zenoh](https://zenoh.io/) to enable ros2_control controllers to interface with hardware running [Zephyr RTOS](https://www.zephyrproject.org/).

- **Position command interface** (write to Zephyr)
- **Position/Velocity/Effort state interfaces** (read from Zephyr)
- **Latency measurement** for performance monitoring

## Dependencies

- ROS 2 (humble+)
- [ros2_control](https://github.com/ros-controls/ros2_control)
- [zenoh](https://github.com/eclipse-zenoh/zenoh) (zenohc, zenohcxx)
- rclcpp, pluginlib, std_msgs

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select zenoh_zephyr_hardware_interface
source install/setup.bash
```

## Configuration

### URDF Hardware Parameters

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `zenoh_endpoint` | string | Yes | Zenoh router endpoint (e.g., `udp/127.0.0.1:7447`) |
| `joint_history_topic` | string | Yes | Zenoh topic for receiving joint states |
| `joint_command_topic` | string | Yes | Zenoh topic for sending joint commands |
| `zenoh_mode` | string | No | Zenoh mode (default: `client`) |

### Example URDF

```xml
<hardware>
  <plugin>zenoh_zephyr_hardware_interface/ZenohZephyrHardware</plugin>
  <param name="zenoh_endpoint">udp/127.0.0.1:7447</param>
  <param name="joint_history_topic">zephyr/joint_states</param>
  <param name="joint_command_topic">zephyr/joint_commands</param>
</hardware>

<joint name="joint_1">
  <command_interface>position</command_interface>
  <state_interface>position</state_interface>
  <state_interface>velocity</state_interface>
  <state_interface>effort</state_interface>
</joint>
```

### Example Controller Configuration

See `examples/zenzep_controllers.yaml` for controller manager configuration using `forward_command_controller`.

## License

Apache-2.0