#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
  // 初始化 ROS 并创建节点
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_target",  // 节点名称为 "move_target"
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));  // 自动声明参数，允许从命令行或文件中覆盖

  // 创建 ROS 日志器
  auto const logger = rclcpp::get_logger("move_target");  // 日志器，用于输出日志信息

  // 创建 MoveIt MoveGroup 接口
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");  // "arm" 是规划组的名称

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

  const double dx = get_double_parameter("dx", 0.1);
  const double dy = get_double_parameter("dy", 0.0);
  const double dz = get_double_parameter("dz", 0.2);

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

  auto target_pose = move_group_interface.getCurrentPose();
  RCLCPP_INFO(
      logger,
      "Current pose: frame=%s, x=%.3f, y=%.3f, z=%.3f",
      target_pose.header.frame_id.c_str(),
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z);

  target_pose.pose.position.x += dx;
  target_pose.pose.position.y += dy;
  target_pose.pose.position.z += dz;

  RCLCPP_INFO(
      logger,
      "Target offset: dx=%.3f, dy=%.3f, dz=%.3f. Target pose: x=%.3f, y=%.3f, z=%.3f",
      dx,
      dy,
      dz,
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z);

  move_group_interface.setPoseTarget(target_pose);  // 将目标位姿设置为 MoveGroup 的目标

  // 生成到目标位姿的规划
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;  // 创建规划消息对象
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));  // 调用规划函数，并检查是否成功
    return std::make_pair(ok, msg);  // 返回规划是否成功及规划消息
  }();

  // 如果规划成功，执行路径
  if (success)
  {
    move_group_interface.execute(plan);  // 执行规划路径
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");  // 如果规划失败，输出错误日志信息
  }

  // 关闭 ROS
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
