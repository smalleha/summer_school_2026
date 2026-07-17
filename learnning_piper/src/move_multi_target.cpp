#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

int main(int argc, char *argv[])
{
  // 初始化 ROS 并创建节点
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_multi_target",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // 创建 ROS 日志器
  auto const logger = rclcpp::get_logger("move_multi_target");

  // 创建 MoveIt MoveGroup 接口，用于与机械臂交互
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const auto get_double_parameter =
      [&node](const std::string &name, const double default_value) {
        if (!node->has_parameter(name)) {
          node->declare_parameter<double>(name, default_value);
        }
        return node->get_parameter(name).get_value<double>();
      };

  const double eef_step = get_double_parameter("eef_step", 0.01);
  const double jump_threshold = get_double_parameter("jump_threshold", 0.0);
  const double min_fraction = get_double_parameter("min_fraction", 0.95);

  move_group_interface.startStateMonitor(5.0);
  auto current_state = move_group_interface.getCurrentState(5.0);
  if (!current_state) {
    RCLCPP_ERROR(
        logger,
        "Failed to receive current robot state. Check /joint_states, controller state, and clock settings.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  move_group_interface.setStartStateToCurrentState();

  const auto current_pose_stamped = move_group_interface.getCurrentPose();
  RCLCPP_INFO(
      logger,
      "Current pose: frame=%s, x=%.3f, y=%.3f, z=%.3f",
      current_pose_stamped.header.frame_id.c_str(),
      current_pose_stamped.pose.position.x,
      current_pose_stamped.pose.position.y,
      current_pose_stamped.pose.position.z);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose = current_pose_stamped.pose;

  target_pose.position.z += 0.2;
  waypoints.push_back(target_pose);

  target_pose.position.x += 0.1;
  waypoints.push_back(target_pose);

  target_pose.position.y += 0.1;
  waypoints.push_back(target_pose);

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    RCLCPP_INFO(
        logger,
        "Cartesian waypoint %zu: x=%.3f, y=%.3f, z=%.3f",
        i + 1,
        waypoints[i].position.x,
        waypoints[i].position.y,
        waypoints[i].position.z);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction =
      move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(logger, "Cartesian path planning fraction: %.2f%%", fraction * 100.0);

  if (fraction < min_fraction) {
    RCLCPP_ERROR(
        logger,
        "Cartesian path planning failed: only %.2f%% completed, required %.2f%%.",
        fraction * 100.0,
        min_fraction * 100.0);
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  cartesian_plan.trajectory_ = trajectory;

  RCLCPP_INFO(logger, "Executing Cartesian path...");
  const auto execute_result = move_group_interface.execute(cartesian_plan);
  if (!static_cast<bool>(execute_result)) {
    RCLCPP_ERROR(logger, "Cartesian path execution failed!");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Cartesian path execution finished.");

  // 关闭 ROS
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
