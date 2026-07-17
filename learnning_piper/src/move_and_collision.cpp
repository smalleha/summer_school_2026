#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_and_collision",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("move_and_collision");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const std::string collision_object_id = "box_obstacle";

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const auto remove_collision_object = [&planning_scene_interface, &collision_object_id, &logger]() {
    planning_scene_interface.removeCollisionObjects({collision_object_id});
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(logger, "Removed collision object: %s", collision_object_id.c_str());
  };

  remove_collision_object();

  const auto shutdown = [&executor, &spinner, &remove_collision_object](const bool cleanup_collision_object) {
    if (cleanup_collision_object) {
      remove_collision_object();
    }
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

  const double dx = get_double_parameter("dx", 0.1);
  const double dy = get_double_parameter("dy", 0.0);
  const double dz = get_double_parameter("dz", 0.4);
  const double box_size_x = get_double_parameter("box_size_x", 0.18);
  const double box_size_y = get_double_parameter("box_size_y", 0.08);
  const double box_size_z = get_double_parameter("box_size_z", 0.10);

  move_group_interface.setPlanningTime(10.0);
  move_group_interface.startStateMonitor(5.0);

  auto current_state = move_group_interface.getCurrentState(5.0);
  if (!current_state) {
    RCLCPP_ERROR(
        logger,
        "Failed to receive current robot state. Check /joint_states, controller state, and clock settings.");
    shutdown(false);
    return 1;
  }

  move_group_interface.setStartStateToCurrentState();

  auto target_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(
      logger,
      "Current pose: frame=%s, x=%.3f, y=%.3f, z=%.3f",
      target_pose.header.frame_id.c_str(),
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z);

  const double start_x = target_pose.pose.position.x;
  const double start_y = target_pose.pose.position.y;
  const double start_z = target_pose.pose.position.z;

  target_pose.pose.position.x += dx;
  target_pose.pose.position.y += dy;
  target_pose.pose.position.z += dz;

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object.id = collision_object_id;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {box_size_x, box_size_y, box_size_z};

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = start_x + dx;
  box_pose.position.y = start_y + dy;
  box_pose.position.z = start_z + dz;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

  if (!planning_scene_interface.applyCollisionObject(collision_object)) {
    RCLCPP_ERROR(logger, "Failed to add collision object to planning scene.");
    shutdown(true);
    return 1;
  }

  RCLCPP_INFO(
      logger,
      "Added cuboid obstacle in frame=%s: center=(%.3f, %.3f, %.3f), size=(%.3f, %.3f, %.3f)",
      collision_object.header.frame_id.c_str(),
      box_pose.position.x,
      box_pose.position.y,
      box_pose.position.z,
      box_size_x,
      box_size_y,
      box_size_z);

  RCLCPP_INFO(
      logger,
      "Target offset: dx=%.3f, dy=%.3f, dz=%.3f. Target pose: x=%.3f, y=%.3f, z=%.3f",
      dx,
      dy,
      dz,
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z);

  move_group_interface.setPoseTarget(target_pose);

  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success) {
    RCLCPP_INFO(logger, "Planning succeeded. Executing trajectory while avoiding the obstacle...");
    const auto execute_result = move_group_interface.execute(plan);
    if (!static_cast<bool>(execute_result)) {
      RCLCPP_ERROR(logger, "Trajectory execution failed!");
      shutdown(true);
      return 1;
    }
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
    shutdown(true);
    return 1;
  }

  RCLCPP_INFO(logger, "Motion finished.");

  shutdown(true);
  return 0;
}
