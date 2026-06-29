# Nav2 Planner / Controller / Costmap-Layer Plugins

Nav2 servers load algorithms through `pluginlib`. To add a global planner,
local controller, or costmap layer you implement the matching `nav2_core` (or
`nav2_costmap_2d`) abstract class, export it with `PLUGINLIB_EXPORT_CLASS`, ship
a plugin-description XML, and select it by name in the server's YAML. The
lifecycle hooks (`configure`/`activate`/`deactivate`/`cleanup`) mirror the host
server's lifecycle transitions — never do heavy work in the constructor.

## Global planner — `nav2_core::GlobalPlanner`

Interface: `configure`, `activate`, `deactivate`, `cleanup`, and the worker
`createPlan(start, goal)` → `nav_msgs::msg::Path`. Loaded by `planner_server`.

```cpp
#include "nav2_core/global_planner.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace my_planner
{
class StraightLine : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent.lock();
    name_ = name;
    global_frame_ = costmap_ros->getGlobalFrameID();
    // Declare params under this plugin's namespace: <name>.interpolation_resolution
    nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".interpolation_resolution", resolution_);
  }
  void activate() override {}
  void deactivate() override {}
  void cleanup() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> /*cancel_checker*/) override   // 3-arg form in Jazzy+
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = node_->now();
    const double dx = goal.pose.position.x - start.pose.position.x;
    const double dy = goal.pose.position.y - start.pose.position.y;
    const int n = std::hypot(dx, dy) / resolution_;
    for (int i = 0; i < n; ++i) {
      geometry_msgs::msg::PoseStamped p = start;
      p.pose.position.x += dx * i / n;
      p.pose.position.y += dy * i / n;
      p.pose.orientation = goal.pose.orientation;
      path.poses.push_back(p);
    }
    path.poses.push_back(goal);
    return path;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_, global_frame_;
  double resolution_;
};
}  // namespace my_planner

PLUGINLIB_EXPORT_CLASS(my_planner::StraightLine, nav2_core::GlobalPlanner)
```

> Note the `createPlan` signature gained a `cancel_checker` argument in Jazzy;
> Humble is the 2-arg form. Match your distro.

## Controller — `nav2_core::Controller`

Interface adds `setPlan(path)`, `computeVelocityCommands(pose, velocity,
goal_checker)` → `geometry_msgs::msg::TwistStamped`, and `setSpeedLimit(limit,
is_percentage)`. Loaded by `controller_server`, ticked at `controller_frequency`.

```cpp
#include "nav2_core/controller.hpp"

class MyController : public nav2_core::Controller
{
public:
  void setPlan(const nav_msgs::msg::Path & path) override { global_plan_ = path; }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & /*velocity*/,
    nav2_core::GoalChecker * /*goal_checker*/) override
  {
    // Pure-pursuit-style: steer toward the first plan point beyond lookahead.
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.frame_id = pose.header.frame_id;
    cmd.header.stamp = clock_->now();
    cmd.twist.linear.x = desired_linear_vel_;
    cmd.twist.angular.z = computeCurvature(pose) * desired_linear_vel_;
    return cmd;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override
  { /* clamp desired_linear_vel_ */ }
  // configure/activate/deactivate/cleanup as in the planner
};
PLUGINLIB_EXPORT_CLASS(my_controller::MyController, nav2_core::Controller)
```

## Costmap layer — `nav2_costmap_2d::Layer`

Interface: `onInitialize`, `updateBounds` (declare the dirty window),
`updateCosts` (write into the master grid), plus `reset`, `matchSize`,
`isClearable`. Loaded by each costmap node; layer order in YAML is composite order.

```cpp
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

class GradientLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node_.lock()->get_parameter(name_ + ".enabled", enabled_);
    need_recalculation_ = false;
  }

  void updateBounds(double, double, double, double * min_x, double * min_y,
                    double * max_x, double * max_y) override
  {
    // Mark the whole map dirty (a real sensor layer would bound to its FOV).
    *min_x = -10.0; *min_y = -10.0; *max_x = 10.0; *max_y = 10.0;
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j,
                   int max_i, int max_j) override
  {
    if (!enabled_) { return; }
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        master.setCost(i, j, (i + j) % 256);  // example gradient
      }
    }
  }
  bool isClearable() override { return false; }

private:
  bool need_recalculation_;
};
PLUGINLIB_EXPORT_CLASS(my_layers::GradientLayer, nav2_costmap_2d::Layer)
```

## Pluginlib export XML + CMake

`my_nav_plugins.xml`:

```xml
<library path="my_nav_plugins">
  <class type="my_planner::StraightLine" base_class_type="nav2_core::GlobalPlanner">
    <description>Straight-line interpolating global planner.</description>
  </class>
  <class type="my_layers::GradientLayer" base_class_type="nav2_costmap_2d::Layer">
    <description>Example gradient costmap layer.</description>
  </class>
</library>
```

`CMakeLists.txt`:

```cmake
add_library(my_nav_plugins SHARED src/straight_line.cpp src/gradient_layer.cpp)
ament_target_dependencies(my_nav_plugins
  nav2_core nav2_costmap_2d pluginlib rclcpp_lifecycle geometry_msgs nav_msgs)
pluginlib_export_plugin_description_file(nav2_core my_nav_plugins.xml)
pluginlib_export_plugin_description_file(nav2_costmap_2d my_nav_plugins.xml)
install(TARGETS my_nav_plugins DESTINATION lib)
```

## YAML wiring

```yaml
planner_server:
  ros__parameters:
    planner_plugins: [GridBased]
    GridBased:
      plugin: my_planner::StraightLine     # selects the exported class
      interpolation_resolution: 0.05

controller_server:
  ros__parameters:
    controller_plugins: [FollowPath]
    FollowPath:
      plugin: my_controller::MyController

local_costmap:
  local_costmap:
    ros__parameters:
      plugins: [obstacle_layer, gradient_layer, inflation_layer]   # order = composite order
      gradient_layer:
        plugin: my_layers::GradientLayer
        enabled: true
```

## Common mistakes

- **Heavy work in the constructor** — allocate ROS resources in `configure`/
  `onInitialize`, not the ctor; the server constructs the plugin before its
  own lifecycle is up.
- **Missing `pluginlib_export_plugin_description_file`** — the class compiles
  but `planner_server` reports "Failed to create global planner. Exception: ...
  not found".
- **Param without the `<name>.` prefix** — plugin params are namespaced by the
  instance name in YAML (`GridBased.interpolation_resolution`); declaring a bare
  key collides across plugins.
- **Wrong `createPlan` arity for the distro** — the cancel-checker overload
  (Jazzy+) vs the 2-arg form (Humble) fails to override and the base throws.
- **Layer not added to `plugins:`** — a registered layer that isn't listed (in
  order) never runs.
