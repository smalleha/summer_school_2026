#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_straight_line",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("move_straight_line");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");
  auto display_trajectory_publisher =
      node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
          "display_planned_path",
          rclcpp::QoS(1).transient_local());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const auto shutdown = [&executor, &spinner]() {
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
  };

  const auto get_double_parameter =
      [&node](const std::string &name, const double default_value) {
        if (!node->has_parameter(name)) {
          node->declare_parameter<double>(name, default_value);
        }
        return node->get_parameter(name).get_value<double>();
      };

  const auto get_bool_parameter =
      [&node](const std::string &name, const bool default_value) {
        if (!node->has_parameter(name)) {
          node->declare_parameter<bool>(name, default_value);
        }
        return node->get_parameter(name).get_value<bool>();
      };

  const double dx = get_double_parameter("dx", 0.0);
  const double dy = get_double_parameter("dy", 0.13);
  const double dz = get_double_parameter("dz", 0.0);
  const double eef_step = get_double_parameter("eef_step", 0.01);
  const double jump_threshold = get_double_parameter("jump_threshold", 0.0);
  const double min_fraction = get_double_parameter("min_fraction", 0.95);
  const double display_time = get_double_parameter("display_time", 2.0);
  const bool execute_motion = get_bool_parameter("execute", true);

  move_group_interface.startStateMonitor(5.0);
  auto current_state = move_group_interface.getCurrentState(5.0);
  if (!current_state) {
    RCLCPP_ERROR(
        logger,
        "Failed to receive current robot state. Check /joint_states, controller state, and clock settings.");
    shutdown();
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

  geometry_msgs::msg::Pose target_pose = current_pose_stamped.pose;
  target_pose.position.x += dx;
  target_pose.position.y += dy;
  target_pose.position.z += dz;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);

  RCLCPP_INFO(
      logger,
      "Straight-line target offset: dx=%.3f, dy=%.3f, dz=%.3f. Target pose: x=%.3f, y=%.3f, z=%.3f",
      dx,
      dy,
      dz,
      target_pose.position.x,
      target_pose.position.y,
      target_pose.position.z);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction =
      move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  RCLCPP_INFO(logger, "Straight-line Cartesian planning fraction: %.2f%%", fraction * 100.0);

  if (fraction < min_fraction) {
    RCLCPP_ERROR(
        logger,
        "Straight-line planning failed: only %.2f%% completed, required %.2f%%.",
        fraction * 100.0,
        min_fraction * 100.0);
    shutdown();
    return 1;
  }

  moveit_msgs::msg::DisplayTrajectory display_trajectory;
  moveit::core::robotStateToRobotStateMsg(*current_state, display_trajectory.trajectory_start);
  display_trajectory.trajectory.push_back(trajectory);
  display_trajectory_publisher->publish(display_trajectory);
  RCLCPP_INFO(logger, "Published planned trajectory to display_planned_path.");

  if (display_time > 0.0) {
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(display_time)));
  }

  if (!execute_motion) {
    RCLCPP_INFO(logger, "execute=false, trajectory display finished without moving the robot.");
    shutdown();
    return 0;
  }

  moveit::planning_interface::MoveGroupInterface::Plan straight_line_plan;
  straight_line_plan.trajectory_ = trajectory;

  RCLCPP_INFO(logger, "Executing straight-line Cartesian trajectory...");
  const auto execute_result = move_group_interface.execute(straight_line_plan);
  if (!static_cast<bool>(execute_result)) {
    RCLCPP_ERROR(logger, "Straight-line trajectory execution failed!");
    shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Straight-line trajectory execution finished.");
  shutdown();
  return 0;
}
