# Configuring a Robot for MoveIt 2

Wiring a URDF robot into MoveIt 2. The MoveIt Setup Assistant generates a
`<robot>_moveit_config` package; these are the files it produces and the ones you hand-
tune. Targets ROS 2 Humble/Jazzy.

## What the Setup Assistant generates

Run `ros2 launch moveit_setup_assistant setup_assistant.launch.py`, load your URDF, and
it writes a `<robot>_moveit_config` package containing:

```
<robot>_moveit_config/
  config/
    <robot>.srdf              # planning groups, end effectors, named poses, ACM
    kinematics.yaml           # IK solver per group
    ompl_planning.yaml        # OMPL planner configs
    joint_limits.yaml         # velocity/acceleration limits for time parameterization
    moveit_controllers.yaml   # controller manager + follow_joint_trajectory mapping
    initial_positions.yaml
  launch/                     # move_group.launch.py, demo.launch.py, rviz, etc.
```

You can regenerate any time by reopening the package in the Setup Assistant — it preserves
hand edits where possible, but review diffs.

## SRDF: planning groups and end effector

The SRDF (semantic robot description) defines what MoveIt can plan for. A **planning
group** is the kinematic chain from base to tip; named states are reusable joint targets.

```xml
<robot name="my_arm">
  <group name="manipulator">
    <chain base_link="base_link" tip_link="tool0"/>
  </group>
  <group name="gripper">
    <link name="left_finger"/>
    <link name="right_finger"/>
  </group>

  <group_state name="ready" group="manipulator">
    <joint name="joint_1" value="0"/>
    <joint name="joint_2" value="-1.2"/>
    <joint name="joint_3" value="1.0"/>
    <joint name="joint_4" value="0"/>
    <joint name="joint_5" value="1.0"/>
    <joint name="joint_6" value="0"/>
  </group_state>

  <end_effector name="eef" parent_link="tool0"
                group="gripper" parent_group="manipulator"/>

  <!-- Disabled collision matrix: skip pairs that never collide -->
  <disable_collisions link1="link_5" link2="link_6" reason="Adjacent"/>
  <disable_collisions link1="base_link" link2="link_1" reason="Adjacent"/>
</robot>
```

Keep the disabled-collision matrix tight — over-disabling hides real self-collisions; the
Setup Assistant computes a safe default by sampling, so re-run it after URDF changes.

## kinematics.yaml: the IK solver

Each planning group gets an IK plugin. KDL is the always-available numeric default;
TRAC-IK converges faster and is the common upgrade.

```yaml
manipulator:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.05
  kinematics_solver_attempts: 3

# TRAC-IK alternative (install trac_ik_kinematics_plugin):
# manipulator:
#   kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
#   kinematics_solver_timeout: 0.05
#   solve_type: Distance
```

A too-short `kinematics_solver_timeout` is a frequent cause of "planning failed" on
reachable poses — raise it before assuming the target is unreachable.

## ompl_planning.yaml: planner configs

Defines which OMPL planners are available and their defaults. RRTConnect is the reliable
general-purpose default for arms.

```yaml
planner_configs:
  RRTConnect:
    type: geometric::RRTConnect
    range: 0.0          # 0 => auto step size
  RRTstar:
    type: geometric::RRTstar
    range: 0.0

manipulator:
  default_planner_config: RRTConnect
  planner_configs:
    - RRTConnect
    - RRTstar
  projection_evaluator: joints(joint_1,joint_2)
  longest_valid_segment_fraction: 0.005   # collision-check resolution along edges
```

Lowering `longest_valid_segment_fraction` checks collisions more finely (safer, slower);
raising it risks tunneling through thin obstacles.

## moveit_controllers.yaml: execution handoff

Maps MoveIt to the `ros2_control` controller that executes the trajectory. The
`action_ns` plus controller name must match the running `follow_joint_trajectory` action.

```yaml
moveit_simple_controller_manager:
  controller_names:
    - joint_trajectory_controller

  joint_trajectory_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    default: true
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
```

The `joints` list must match the controller's own `joints` order and the URDF — a
mismatch silently maps trajectory points to the wrong joints.

## joint_limits.yaml: time parameterization

MoveIt re-times every trajectory under these limits. They can be stricter than the URDF
(e.g. to slow the robot for safety) but never looser.

```yaml
joint_limits:
  joint_1:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: true
    max_acceleration: 4.0
```

## Bring-up checklist

1. `ros2 control list_controllers` — confirm `joint_trajectory_controller` is `active`.
2. Launch `move_group` (from the generated `move_group.launch.py`); check no
   kinematics/controller load errors in the log.
3. In RViz MotionPlanning, drag the interactive marker, Plan, then Execute — confirms the
   full SRDF → planner → controller chain end to end before running application code.
