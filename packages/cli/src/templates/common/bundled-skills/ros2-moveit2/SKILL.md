---
name: ros2-moveit2
description: Use MoveIt 2 for ROS 2 arm motion planning. Reach for it on signals like "MoveIt", "MoveIt2", "motion planning", "manipulator/arm planning", "MoveGroupInterface", "planning scene", "collision checking", "OMPL", "inverse kinematics for an arm", "joint/pose target", "Cartesian path", "pick and place".
---

# ros2-moveit2

MoveIt 2 is the motion-planning framework for ROS 2 manipulators. Reach for this skill when planning collision-free arm trajectories, querying IK, managing the planning scene, or wiring MoveIt to `ros2_control` execution. The default stack is a URDF/SRDF robot description, the `move_group` node, OMPL/Pilz/CHOMP planners, and a `joint_trajectory_controller` that executes the planned trajectory.

Typical user signals: "plan a motion to this pose", "MoveGroupInterface won't reach the target", "add a collision object to the planning scene", "set up MoveIt for my arm", "configure OMPL / kinematics.yaml", "Cartesian straight-line path", "pick and place with MoveIt", "IK keeps failing", "the controller rejects the trajectory".

This skill is an index. Load only the reference file for the current job — do not preload both.

## Architecture in one screen

- **`move_group`** — the central node. Aggregates the robot model (URDF), semantics (SRDF), kinematics, planners, and the planning scene; exposes planning/execution actions and services that `MoveGroupInterface` talks to.
- **Planning pipeline** — request → planner → adapters (time parameterization, fix-start-state, resolve-constraints) → trajectory. Planners: **OMPL** (sampling-based, general purpose), **Pilz Industrial Motion Planner** (deterministic LIN/PTP/CIRC), **CHOMP/STOMP** (optimization-based).
- **Planning Scene Monitor** — the live world model: robot state + collision objects (`moveit_msgs/PlanningScene`). Plans are only as correct as this scene; a stale scene means the arm plans through obstacles.
- **Collision checking** — FCL-based; uses the SRDF disabled-collision matrix to skip adjacent links. Use convex collision meshes/primitives, never full visual meshes.
- **Kinematics (IK/FK)** — pluggable solver per planning group. Defaults: KDL (numeric, always available) or TRAC-IK (faster convergence). Analytic solvers (e.g. IKFast) when available.

## Two ways to drive it

| API | Use when |
|---|---|
| `MoveGroupInterface` (C++/Python) | Application code: set a pose/joint target, `plan()`, `execute()`, Cartesian paths, scene edits. Talks to `move_group` over actions. **Start here.** |
| Planning pipeline / `PlanningComponent` (`moveit_cpp`) | In-process, low-latency, multi-query planning without the action round-trip; advanced control over scene and planner config. |

## SRDF, planning groups, planning scene

- A **planning group** is the kinematic chain `base_link → tip_link` defined in the SRDF (e.g. `manipulator`). MoveIt plans for a group, and IK/collision are scoped to it.
- The SRDF also declares the **end effector**, **named poses** (`ready`, `home`), and the **disabled-collision matrix** (pairs that never collide). Targets are expressed in `getPlanningFrame()`.
- The **planning scene** is mutable at runtime via `PlanningSceneInterface`: add obstacles before planning, and `attachObject` a grasped payload so it is treated as part of the robot.

## Controller handoff

MoveIt does not move motors. It produces a **time-parameterized** `trajectory_msgs/JointTrajectory` and sends it to a `ros2_control` `joint_trajectory_controller` through the `follow_joint_trajectory` action (configured in `moveit_controllers.yaml`). The controller's `joints` order must match the order MoveIt sends, or commands map to the wrong joints. Verify with `ros2 control list_controllers` that the controller is `active` before executing.

## Core rules

- Always plan against an **up-to-date planning scene**; edit it before calling `plan()`.
- Keep velocity/acceleration scaling conservative (`setMaxVelocityScalingFactor(0.2–0.3)`) on first runs — scaling re-times the trajectory under URDF/`joint_limits.yaml`.
- A failed `plan()` is usually IK failure (unreachable / in collision), a bad start state, or a too-short planning/IK timeout — check those before blaming the planner.
- Separate `plan()` from `execute()` so a bad plan can be inspected or rejected; use `move()` only for trusted, simple goals.

## Load only the reference file you need

- `references/motion-planning.md` — writing the C++ planning code: `MoveGroupInterface` pose/joint targets, plan+execute, Cartesian paths, collision objects, path constraints, IK queries.
- `references/setup-and-config.md` — configuring a robot for MoveIt: Setup Assistant outputs, SRDF planning groups/end effectors, `kinematics.yaml`, `ompl_planning.yaml`, `moveit_controllers.yaml`.

## Not for

- Mobile-base navigation (use Nav2) or wheeled/legged locomotion.
- Low-level `ros2_control` controller authoring — MoveIt only hands off the trajectory; controller internals are a separate concern.
- Pure forward-dynamics simulation; MoveIt plans kinematic/collision-aware paths, it is not a physics engine.
