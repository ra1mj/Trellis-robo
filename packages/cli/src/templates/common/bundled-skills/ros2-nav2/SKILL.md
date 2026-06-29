---
name: ros2-nav2
description: Develop and configure ROS 2 Nav2 navigation — behavior tree navigator (bt_navigator), planner/controller/behavior servers, costmap2d global/local layers, lifecycle management, AMCL, and custom planner/controller/costmap-layer or BT-node plugins. Use when the user works on Nav2, navigation, path planning, costmaps, behavior trees, global/local planners, or recovery behaviors.
---

# ROS 2 Nav2

Nav2 is the ROS 2 navigation stack: a set of managed-lifecycle servers wired
together by a behavior tree. Reach for this skill when the user mentions Nav2,
"navigation", "costmap", "path planning", "behavior tree navigator",
"bt_navigator", a "planner/controller plugin", "AMCL", "global/local planner",
or recovery behaviors.

This skill is an index. Load only the reference file for the current job — do
not preload both.

## Architecture

Nav2 is **not** one node. It is a graph of single-responsibility lifecycle
servers, each loading pluginlib plugins, orchestrated by a behavior tree.

| Server | Node | Action server | Plugins it loads |
|--------|------|---------------|------------------|
| BT Navigator | `bt_navigator` | `NavigateToPose`, `NavigateThroughPoses` | BT node plugins + the BT XML |
| Planner | `planner_server` | `ComputePathToPose` | `nav2_core::GlobalPlanner` (NavFn, SmacHybrid, ThetaStar) |
| Controller | `controller_server` | `FollowPath` | `nav2_core::Controller` (DWB, RPP, MPPI) + goal/progress checkers |
| Behavior | `behavior_server` | `Spin`, `BackUp`, `Wait`, `DriveOnHeading` | `nav2_core::Behavior` |
| Smoother | `smoother_server` | `SmoothPath` | `nav2_core::Smoother` |
| Costmaps | `global_costmap`, `local_costmap` | — | `nav2_costmap_2d::Layer` (static, obstacle, voxel, inflation) |
| Localization | `amcl`, `map_server` | — | — |

`global_costmap` is rolling-window-off in `map` frame (whole map); `local_costmap`
is a rolling window in `odom` frame for reactive obstacle avoidance. Each costmap
is a layered grid — the **order** of layers in YAML is the composition order.

## Lifecycle management

Every Nav2 server is a managed lifecycle node (unconfigured → inactive →
active). `nav2_lifecycle_manager` transitions them **in order** and holds a
`bond` connection to each so a crashed server is detected and the whole stack
is brought down rather than left half-up.

```yaml
lifecycle_manager:
  ros__parameters:
    autostart: true
    bond_timeout: 4.0
    node_names: [map_server, amcl, controller_server, smoother_server,
                 planner_server, behavior_server, bt_navigator, waypoint_follower]
```

Order matters: `map_server`/`amcl` before the servers that read the costmap,
`bt_navigator` last because it calls the others. A node missing from
`node_names` never activates and its action server is silently absent.

## The plugin model

Nav2 is customized through plugins, not forks. Algorithms (global planner, local
controller, costmap layer, BT node, goal checker, smoother, behavior) are
`pluginlib` classes selected by `plugin:` strings in YAML. You write a class
implementing the abstract interface, export it with `PLUGINLIB_EXPORT_CLASS`,
ship a plugin-description XML, and reference it by name in the server's params.

## Key config / files

- `nav2_params.yaml` — one big param file, namespaced per server; the
  `plugin:` keys decide which algorithms load.
- The BT XML (`default_nav_to_pose_bt_xml`) — the navigation logic.
- `<pkg>_plugin.xml` + `pluginlib` export — registers a custom plugin.
- Launch: `nav2_bringup` composes the servers; a `composable_node` container
  runs them in one process for zero-copy.

## TF / odom requirements (hard prerequisites)

Nav2 will not start navigating without an unbroken TF chain:

```
map --(amcl/slam)--> odom --(odom source)--> base_link --(URDF static)--> sensors
```

- `global_costmap.global_frame: map`, `robot_base_frame: base_link`.
- `local_costmap.global_frame: odom` — must update smoothly and never jump.
- A `nav_msgs/Odometry` source on `/odom` and a valid `sensor_msgs/LaserScan`
  (or pointcloud) feeding the obstacle layer.
- `transform_tolerance` must exceed real TF latency or the costmap stalls.

## Load only the reference you need

- `references/behavior-trees.md` — BT XML structure, Nav2 control/action/decorator
  nodes, recovery patterns, blackboard, and writing a **custom BT node plugin** in
  C++ (`BtActionNode`, `providedPorts`, `BT_REGISTER_NODES`).
- `references/plugins.md` — writing **planner / controller / costmap-layer**
  plugins: the `nav2_core::GlobalPlanner` / `nav2_core::Controller` /
  `nav2_costmap_2d::Layer` interfaces, `pluginlib` export, and YAML wiring.

## Not for

- Generic differential-drive kinematics, odometry fusion, or `ros2_control`
  tuning — those live in the mobile-robot domain spec.
- SLAM / mapping internals (slam_toolbox, cartographer) beyond the TF contract.
