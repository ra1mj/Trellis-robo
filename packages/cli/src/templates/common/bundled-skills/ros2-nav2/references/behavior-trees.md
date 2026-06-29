# Nav2 Behavior Trees

Nav2's `bt_navigator` runs a [BehaviorTree.CPP](https://www.behaviortree.dev)
tree on every `NavigateToPose` goal. The tree is the navigation logic — it
calls the planner/controller/recovery action servers as leaf nodes. Edit the
XML to change *behavior*; write a custom BT node to add a new *capability*.

BehaviorTree.CPP v3 ships with Humble; v4 with Jazzy/Rolling. Port syntax and
the `<root>` attribute differ between them (`main_tree_to_execute` →
`BTCPP_format="4"`), so match the BT lib version of your distro.

## XML structure

Tick order is top-to-bottom, left-to-right. A node returns SUCCESS / FAILURE /
RUNNING; control nodes route on those.

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1.0">
          <ComputePathToPose goal="{goal}" path="{path}"
                             planner_id="GridBased"/>
        </RateController>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </PipelineSequence>
      <!-- recovery branch: ticked only when the main branch FAILS -->
      <RoundRobin name="RecoveryActions">
        <ClearEntireCostmap name="ClearLocal"
                            service_name="local_costmap/clear_entirely_local_costmap"/>
        <ClearEntireCostmap name="ClearGlobal"
                            service_name="global_costmap/clear_entirely_global_costmap"/>
        <Spin spin_dist="1.57"/>
        <BackUp backup_dist="0.30" backup_speed="0.05"/>
        <Wait wait_duration="5.0"/>
      </RoundRobin>
    </RecoveryNode>
  </BehaviorTree>
</root>
```

`{goal}` and `{path}` are **blackboard** entries: `{name}` reads/writes the
shared key. `bt_navigator` pre-loads `goal`, `goals`, and pose data onto the
blackboard before the first tick.

## Node families you will use

| Family | Examples | Purpose |
|--------|----------|---------|
| Control | `PipelineSequence`, `RecoveryNode`, `RoundRobin`, `ReactiveFallback`, `Sequence` | Route ticks based on child status |
| Action (Nav2) | `ComputePathToPose`, `FollowPath`, `Spin`, `BackUp`, `DriveOnHeading`, `Wait`, `ClearEntireCostmap` | Wrap Nav2 ROS action/service servers |
| Decorator | `RateController`, `DistanceController`, `SpeedController`, `GoalUpdater`, `SingleTrigger` | Throttle / gate a subtree |
| Condition | `GoalReached`, `GoalUpdated`, `IsBatteryLow`, `TransformAvailable`, `InitialPoseReceived` | Reactive predicates |

Key idioms:

- `PipelineSequence` re-ticks earlier children while later ones run — this is
  what lets the planner replan (via `RateController hz`) while `FollowPath`
  keeps driving.
- `RecoveryNode` runs child[0]; on FAILURE it runs child[1] (recoveries) then
  retries child[0], up to `number_of_retries`.
- `ReactiveFallback` with a `GoalUpdated` condition aborts the current path when
  a new goal arrives.

## Wiring the XML into bt_navigator

```yaml
bt_navigator:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    default_nav_to_pose_bt_xml: $(find-pkg-share my_nav)/behavior_trees/nav_to_pose.xml
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - my_check_zone_condition_bt_node   # your custom node's library
```

Every leaf node must have its registering library listed in `plugin_lib_names`,
or the tree fails to load with "ID not registered".

## Custom BT node (C++)

To wrap a custom ROS action, derive from `nav2_behavior_tree::BtActionNode<ActionT>`.
For a pure predicate, derive from `BT::ConditionNode`.

```cpp
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_util/robot_utils.hpp"

namespace my_nav
{
// Returns SUCCESS when the robot is inside an allowed zone, else FAILURE.
class IsInZoneCondition : public BT::ConditionNode
{
public:
  IsInZoneCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  }

  // Ports declared here are exposed to the XML.
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("max_radius", 5.0, "allowed radius [m]")};
  }

  BT::NodeStatus tick() override
  {
    double max_radius;
    getInput("max_radius", max_radius);
    geometry_msgs::msg::PoseStamped pose;
    if (!nav2_util::getCurrentPose(pose, *tf_, "map", "base_link")) {
      return BT::NodeStatus::FAILURE;
    }
    const double d = std::hypot(pose.pose.position.x, pose.pose.position.y);
    return d <= max_radius ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};
}  // namespace my_nav

// C-style export so bt_navigator's factory can load it from a shared lib.
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<my_nav::IsInZoneCondition>("IsInZone");
}
```

`CMakeLists.txt`: build it as a SHARED library and name the library exactly
what you list in `plugin_lib_names`:

```cmake
add_library(my_check_zone_condition_bt_node SHARED src/is_in_zone_condition.cpp)
ament_target_dependencies(my_check_zone_condition_bt_node
  behaviortree_cpp_v3 nav2_behavior_tree nav2_util rclcpp)
install(TARGETS my_check_zone_condition_bt_node DESTINATION lib)
```

Then use it in XML: `<IsInZone max_radius="4.0"/>`.

## Common mistakes

- **Forgetting `plugin_lib_names`** — the node compiles but `bt_navigator`
  reports the XML ID as unregistered.
- **Blocking in `tick()`** — BT nodes are ticked at the BT loop rate; long work
  belongs in an async action node (`BtActionNode`), never a synchronous spin.
- **Mismatched BT.CPP version** — v3 vs v4 XML (`<root>` attributes, port
  remapping) silently fails to parse on the wrong distro.
- **Using `Sequence` instead of `PipelineSequence`** for plan+follow — a plain
  `Sequence` waits for the planner to return before following, killing replanning.
