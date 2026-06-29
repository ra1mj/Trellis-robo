# Motion Planning with MoveIt 2 (C++)

Planning and executing arm motions with `MoveGroupInterface`. Examples target ROS 2
Humble/Jazzy and C++17. The node must spin on a separate executor thread so action
results arrive while you block on `plan()`/`execute()`.

## Node and MoveGroupInterface setup

`MoveGroupInterface` needs a spinning node to receive the robot state and action
feedback. Spin in a background thread:

```cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

auto node = std::make_shared<rclcpp::Node>(
    "moveit_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
std::thread spinner([&executor]() { executor.spin(); });

using moveit::planning_interface::MoveGroupInterface;
MoveGroupInterface arm(node, "manipulator");   // "manipulator" = SRDF planning group
arm.setPlanningTime(5.0);
arm.setMaxVelocityScalingFactor(0.3);
arm.setMaxAccelerationScalingFactor(0.3);
arm.setNumPlanningAttempts(5);
```

## Pose target: plan then execute

Set a Cartesian goal for the end-effector frame, plan, and only execute on success.
Always separate planning from execution so you can inspect or reject a bad plan.

```cpp
geometry_msgs::msg::Pose target;
target.orientation.w = 1.0;
target.position.x = 0.4; target.position.y = 0.1; target.position.z = 0.5;
arm.setPoseTarget(target);                  // pose in arm.getPlanningFrame()

MoveGroupInterface::Plan plan;
if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
  arm.execute(plan);                        // blocks until the controller finishes
} else {
  RCLCPP_WARN(node->get_logger(), "planning failed (IK/collision/timeout)");
}
```

## Joint-space target

Joint goals skip IK and are more reliable for known configurations (home, stow):

```cpp
std::vector<double> joints = {0.0, -1.2, 1.0, 0.0, 1.0, 0.0};   // radians, JTC order
arm.setJointValueTarget(joints);
MoveGroupInterface::Plan plan;
if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) arm.execute(plan);

// Named target from the SRDF (e.g. "ready" pose):
arm.setNamedTarget("ready");
arm.move();                                  // convenience: plan + execute in one call
```

## Cartesian path (straight-line / waypoints)

`computeCartesianPath` interpolates the end-effector along waypoints — use it for
approach/retreat moves where the tool must travel in a straight line.

```cpp
std::vector<geometry_msgs::msg::Pose> waypoints;
geometry_msgs::msg::Pose wp = arm.getCurrentPose().pose;
wp.position.z -= 0.10; waypoints.push_back(wp);   // 10 cm straight down
wp.position.x += 0.15; waypoints.push_back(wp);   // 15 cm forward

moveit_msgs::msg::RobotTrajectory traj;
const double eef_step = 0.01;                      // 1 cm interpolation resolution
double fraction = arm.computeCartesianPath(waypoints, eef_step, traj);
if (fraction > 0.95) {                             // >95% of the path solvable
  arm.execute(traj);
} else {
  RCLCPP_WARN(node->get_logger(), "only %.0f%% of Cartesian path solved", fraction * 100);
}
```

A low `fraction` means a singularity or joint limit blocked the straight line — replan
with OMPL (which can detour) or change the approach.

## Planning scene: collision objects

Add obstacles so the planner avoids them, and attach grasped payloads so they move with
the arm. Edits go through `PlanningSceneInterface`.

```cpp
#include <moveit/planning_scene_interface/planning_scene_interface.h>
moveit::planning_interface::PlanningSceneInterface psi;

moveit_msgs::msg::CollisionObject box;
box.header.frame_id = arm.getPlanningFrame();
box.id = "table";
shape_msgs::msg::SolidPrimitive prim;
prim.type = prim.BOX;
prim.dimensions = {0.8, 0.8, 0.04};               // x, y, z
geometry_msgs::msg::Pose pose;
pose.orientation.w = 1.0; pose.position.z = 0.2;
box.primitives.push_back(prim);
box.primitive_poses.push_back(pose);
box.operation = box.ADD;
psi.applyCollisionObject(box);

// After grasping, attach the payload to the gripper link so planning accounts for it:
arm.attachObject("part", "tool0");
```

## Path constraints (e.g. keep tool upright)

Orientation/position constraints restrict the plan — useful for carrying an open
container so it stays level. Constraints are slower; clear them after the move.

```cpp
moveit_msgs::msg::OrientationConstraint ocm;
ocm.link_name = "tool0";
ocm.header.frame_id = arm.getPlanningFrame();
ocm.orientation.w = 1.0;
ocm.absolute_x_axis_tolerance = 0.1;
ocm.absolute_y_axis_tolerance = 0.1;
ocm.absolute_z_axis_tolerance = 3.14;             // allow free yaw
ocm.weight = 1.0;

moveit_msgs::msg::Constraints constraints;
constraints.orientation_constraints.push_back(ocm);
arm.setPathConstraints(constraints);
// ... setPoseTarget(...) + plan + execute ...
arm.clearPathConstraints();
```

## Querying IK directly

For reachability checks without planning, use the `RobotState` IK API:

```cpp
moveit::core::RobotStatePtr state = arm.getCurrentState();
const moveit::core::JointModelGroup* jmg =
    state->getJointModelGroup("manipulator");
bool ok = state->setFromIK(jmg, target, 0.1 /*timeout s*/);   // true if reachable
if (ok) { std::vector<double> q; state->copyJointGroupPositions(jmg, q); }
```

## Common failure modes

- **`plan()` returns failure** — target unreachable (IK), start state in collision, or
  planning time too short. Check IK with `setFromIK` and verify the scene.
- **Plan succeeds but `execute()` fails** — controller not running, action server name
  mismatch, or trajectory violates controller goal-time/tolerance constraints.
- **Low Cartesian `fraction`** — singularity or joint limit on the straight line; detour
  with OMPL or adjust waypoints.
- **Arm clips an obstacle** — planning scene was stale; obstacle added after planning, or
  not added at all. Always edit the scene before planning.
