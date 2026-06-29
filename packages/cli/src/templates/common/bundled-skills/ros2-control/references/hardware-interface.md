# Writing a ros2_control hardware interface

A hardware interface owns the physical (or simulated) robot and is loaded by the
controller_manager as a `pluginlib` plugin. Pick the base class by topology:

| Base class | Use for |
|------------|---------|
| `hardware_interface::SystemInterface` | a multi-DOF robot with both command and state (most cases) |
| `hardware_interface::ActuatorInterface` | a single actuator |
| `hardware_interface::SensorInterface` | read-only sensor (state interfaces only) |

This covers `SystemInterface` — the common case. Package deps (`package.xml`):
`hardware_interface`, `pluginlib`, `rclcpp_lifecycle`, `rclcpp`.

## Class skeleton

```cpp
// include/my_robot_hardware/robot_system.hpp
#pragma once
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <vector>

namespace my_robot_hardware {

class RobotSystem : public hardware_interface::SystemInterface {
 public:
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
      const rclcpp::Time & time, const rclcpp::Duration & period) override;

 private:
  std::vector<double> hw_positions_, hw_velocities_;   // state storage
  std::vector<double> hw_commands_;                    // command storage
};

}  // namespace my_robot_hardware
```

## on_init: validate the URDF and size storage

`info_` (a base-class member of type `HardwareInfo`) is parsed from the `<ros2_control>` URDF
tag. Always call the parent first, then size your buffers and validate that each joint
exposes exactly the interfaces you support.

```cpp
hardware_interface::CallbackReturn RobotSystem::on_init(
    const hardware_interface::HardwareInfo & info) {
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::size_t n = info_.joints.size();
  hw_positions_.assign(n, 0.0);                  // allocate ONCE, never in read/write
  hw_velocities_.assign(n, 0.0);
  hw_commands_.assign(n, 0.0);

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(get_logger(), "Joint '%s' needs exactly one position command",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}
```

## Export interfaces: hand storage pointers to the manager

Each exported interface binds a `<joint>/<interface>` name to the address of a `double` you
own. The controller_manager reads those addresses after `read()` and writes them before
`write()`. The storage must outlive activation — they are the vectors sized in `on_init`.

```cpp
std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> ifaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    ifaces.emplace_back(info_.joints[i].name,
                        hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return ifaces;
}
```

## The cyclic read/write contract (real-time)

`read()` and `write()` run inside the controller_manager loop at `update_rate`, on the same
RT thread as controller `update()`. They obey the same real-time discipline — no heap, no
blocking locks, no exceptions, no logging in the steady state. See
`.trellis/spec/robotics/cpp-performance.md`.

- `read()`: pull the latest sensor values from the device into `hw_positions_` /
  `hw_velocities_`. Non-blocking; if the fieldbus has no fresh frame, keep the last value.
- `write()`: push `hw_commands_` (set by the active controller) to the device.
- Open the bus/socket in `on_activate`, close it in `on_deactivate` — never in `read`/`write`.

```cpp
hardware_interface::CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State &) {
  for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_commands_[i] = hw_positions_[i];          // start from current pose, no jump
  }
  // open_can_bus();  // acquire the device here
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotSystem::read(
    const rclcpp::Time &, const rclcpp::Duration & period) {
  for (std::size_t i = 0; i < hw_positions_.size(); ++i) {
    // hw_positions_[i] = bus_.latestPosition(i);  // device read, non-blocking
    hw_velocities_[i] = (hw_commands_[i] - hw_positions_[i]) / period.seconds();
    hw_positions_[i] = hw_commands_[i];            // (demo: perfect actuator)
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotSystem::write(
    const rclcpp::Time &, const rclcpp::Duration &) {
  for (std::size_t i = 0; i < hw_commands_.size(); ++i) {
    // bus_.sendPositionCommand(i, hw_commands_[i]);
  }
  return hardware_interface::return_type::OK;
}
```

## URDF `ros2_control` tag

The hardware is declared in the robot_description (usually via xacro). The controller_manager
parses this into `info_`. Joint interface names here must match what `export_*_interfaces`
publishes and what controllers claim.

```xml
<ros2_control name="RobotSystem" type="system">
  <hardware>
    <plugin>my_robot_hardware/RobotSystem</plugin>
    <param name="can_interface">can0</param>
  </hardware>
  <joint name="joint1">
    <command_interface name="position">
      <param name="min">-3.14</param>
      <param name="max">3.14</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="joint2">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

For simulation, swap the `<plugin>` for `mock_components/GenericSystem` (built-in loopback)
or the Gazebo `gz_ros2_control` plugin — the controllers and URDF interfaces stay identical.

## pluginlib export + CMake

```cpp
// bottom of robot_system.cpp
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    my_robot_hardware::RobotSystem, hardware_interface::SystemInterface)
```

```xml
<!-- my_robot_hardware.xml -->
<library path="my_robot_hardware">
  <class name="my_robot_hardware/RobotSystem"
         type="my_robot_hardware::RobotSystem"
         base_class_type="hardware_interface::SystemInterface">
    <description>System interface for the example robot.</description>
  </class>
</library>
```

```cmake
add_library(my_robot_hardware SHARED src/robot_system.cpp)
ament_target_dependencies(my_robot_hardware hardware_interface pluginlib rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware.xml)
install(TARGETS my_robot_hardware LIBRARY DESTINATION lib)
```

Verify on a running system with `ros2 control list_hardware_components` and
`ros2 control list_hardware_interfaces` — every exported interface should appear as
`available` once the component is activated.
