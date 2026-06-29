---
name: ros2-control
description: Use when writing or debugging ros2_control code — authoring a controller plugin (controller_interface::ControllerInterface), a hardware_interface::SystemInterface, command/state interfaces, controller_manager / ros2_controllers wiring, or a real-time control loop. Triggers on signals like "write a controller", "hardware_interface", "controller_manager", "real-time control loop", "ros2_control.xacro", "export_state_interfaces", "update(time, period)", "RealtimeBuffer".
---

# ros2-control

`ros2_control` is the ROS 2 framework for real-time robot control. It splits a robot into
**controllers** (compute commands from references + state) and **hardware interfaces**
(talk to the physical or simulated robot), brokered by the **controller_manager** which
runs the deterministic `read() → update() → write()` cycle.

Reach for this skill when the user is authoring or debugging anything in that stack: a new
controller, a hardware driver plugin, interface wiring, the `ros2_control` URDF tag, or a
real-time handoff between a ROS callback and the control loop.

This skill is an index. Load only the reference file for the current job — do not preload
both.

## Mental model

The controller_manager runs a deterministic loop at `update_rate`:
`read()` → `controller.update()` → `write()`.

- A **controller** declares which `command_interfaces` it claims and which `state_interfaces`
  it reads, then in `update()` reads state and sets commands. It never touches hardware.
- A **hardware interface** owns the device. `read()` copies sensor values into the state
  storage exported to controllers; `write()` pushes claimed command values to the device.
- The **controller_manager** loads both from plugins, matches interface names by
  `<joint>/<interface>`, enforces the loop rate, and manages lifecycle.

## Controller lifecycle (the methods you implement)

| Method | When | Real-time? | Do |
|--------|------|-----------|-----|
| `on_init()` | once, after load | no | declare params, set up `ParamListener`; return `ERROR` on bad config |
| `command_interface_configuration()` | after configure | no | list claimed `<joint>/<interface>` names (or `ALL`/`NONE`) |
| `state_interface_configuration()` | after configure | no | list read `<joint>/<interface>` names |
| `on_configure()` | inactive | no | read params, allocate buffers, create subs/pubs, **reserve** everything |
| `on_activate()` | activating | no | cache interface handles/indices, zero state |
| `update(time, period)` | every cycle | **YES** | the control law — bounded, no alloc/lock/throw |
| `on_deactivate()` | deactivating | no | stop motion, release handles |

## The `update(time, period)` real-time contract

`update()` runs inside the controller_manager's deterministic loop. It is the single most
constrained function in the codebase. The full discipline is in
`.trellis/spec/robotics/cpp-performance.md` — the non-negotiable subset:

- **No heap**: no `new`/`malloc`, no `std::vector` growth, no `std::string`. Allocate in
  `on_configure`/`on_activate`.
- **No locks shared with non-RT threads** (priority inversion). Hand data across with
  `realtime_tools::RealtimeBuffer<T>` (non-RT writes, RT reads) and publish telemetry with
  `RealtimePublisher<T>` using `trylock()` — drop the sample, never block.
- **No `throw`, no I/O, no logging** (`RCLCPP_INFO` allocates and locks). Use throttled logs
  off the hot path only, or a diagnostic topic.
- **Bounded loops**: iteration counts are compile-time or fixed-capacity, never driven by
  runtime data.

```cpp
controller_interface::return_type MyController::update(
    const rclcpp::Time &, const rclcpp::Duration & period) {
  const Command cmd = *rt_command_.readFromRT();          // by value, fixed size
  for (std::size_t i = 0; i < n_joints_; ++i) {           // bounded
    const double err = cmd.position[i] - state_pos_[i]->get_value();
    command_eff_[i]->set_value(kp_[i] * err);             // write claimed interface
  }
  return controller_interface::return_type::OK;
}
```

## Wiring it together

- **`controller_manager`** is launched with a YAML that sets `update_rate` and maps a
  controller name to its plugin `type`. Spawn with `ros2 run controller_manager spawner <name>`.
- **`ros2_control` URDF/xacro tag** (`<ros2_control name=... type="system">`) declares the
  hardware plugin and each joint's command/state interfaces; the controller_manager parses it.
- **`ros2_controllers`** ships ready-made controllers — `joint_state_broadcaster`,
  `joint_trajectory_controller`, `diff_drive_controller`. Always run `joint_state_broadcaster`.
- **`generate_parameter_library`** turns a YAML schema into a typed, validated `Params`
  struct + `ParamListener` — use it instead of hand-rolled `declare_parameter`.

## Load only the reference you need

| Task | Read |
|------|------|
| Author / fix a **controller plugin** (skeleton, interface config, params, pluginlib export, YAML) | `references/controllers.md` |
| Author / fix a **hardware interface** (`SystemInterface`, read/write, URDF tag, lifecycle) | `references/hardware-interface.md` |

## Not for

- Generic rclcpp node/QoS/parameter conventions — those are in
  `.trellis/spec/robotics/ros2-conventions.md`.
- Motion planning (MoveIt2) or navigation (Nav2) — separate skills/specs.
- Tuning the OS/kernel for real-time (`SCHED_FIFO`, `isolcpus`, `PREEMPT_RT`) — see the
  determinism section of `.trellis/spec/robotics/cpp-performance.md`.
