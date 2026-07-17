#include <array>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using moveit::planning_interface::MoveGroupInterface;

// 1. 提取常量，避免代码中的"魔法数字"
constexpr double kVelocityScaling = 0.2;
constexpr double kAccelerationScaling = 0.2;
constexpr double kStateTimeoutSeconds = 5.0;

bool planAndExecute(
    MoveGroupInterface &move_group_interface,
    const rclcpp::Logger &logger,
    const std::string &motion_name)
{
  MoveGroupInterface::Plan plan;
  
  // 2. 显式使用 MoveItErrorCode，比强转 bool 更严谨和规范
  auto plan_result = move_group_interface.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Planning failed: %s", motion_name.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Executing: %s", motion_name.c_str());
  auto exec_result = move_group_interface.execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Execution failed: %s", motion_name.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "Finished: %s", motion_name.c_str());
  return true;
}

// 3. 使用模板泛型化，完美合并了 moveArmJoints 和 moveGripperJoints
template <std::size_t N>
bool moveJoints(
    MoveGroupInterface &move_group,
    const rclcpp::Logger &logger,
    const std::array<double, N> &joint_positions,
    const std::string &motion_name)
{
  // 使用迭代器进行现代化且简洁的 vector 初始化
  const std::vector<double> target(joint_positions.begin(), joint_positions.end());

  move_group.setStartStateToCurrentState();
  move_group.setJointValueTarget(target);
  return planAndExecute(move_group, logger, motion_name);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_joint",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto const logger = rclcpp::get_logger("move_joint");

  MoveGroupInterface arm_group(node, "arm");
  MoveGroupInterface gripper_group(node, "gripper");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  const auto shutdown = [&executor, &spinner]() {
    executor.cancel();
    if (spinner.joinable()) { // 增加 joinable 检查，提高线程安全性
      spinner.join();
    }
    rclcpp::shutdown();
  };

  arm_group.setMaxVelocityScalingFactor(kVelocityScaling);
  arm_group.setMaxAccelerationScalingFactor(kAccelerationScaling);
  gripper_group.setMaxVelocityScalingFactor(kVelocityScaling);
  gripper_group.setMaxAccelerationScalingFactor(kAccelerationScaling);

  arm_group.startStateMonitor(kStateTimeoutSeconds);
  auto current_state = arm_group.getCurrentState(kStateTimeoutSeconds);
  if (!current_state) {
    RCLCPP_ERROR(
        logger,
        "Failed to receive current robot state. Check /joint_states, controller state, and clock settings.");
    shutdown();
    return 1;
  }

  // 4. 使用 constexpr 声明编译期常量
  constexpr std::array<double, 2> gripper_open = {0.038, -0.038};
  constexpr std::array<double, 2> gripper_close = {0.0, 0.0};

  constexpr std::array<double, 6> pre_grasp = {0.0, 1.42, -1.18, 0.0, 1.21, 0.0};
  constexpr std::array<double, 6> grasp = {0.0, 1.72, -0.81, 0.0, 0.53, 0.0};

  // 5. 利用短路求值 (Short-circuit evaluation) 优化流水线控制流
  const bool success = 
      moveJoints(gripper_group, logger, gripper_open, "open gripper") &&
      moveJoints(arm_group, logger, pre_grasp, "move arm to pre-grasp point") &&
      moveJoints(arm_group, logger, grasp, "move arm to grasp point") &&
      moveJoints(gripper_group, logger, gripper_close, "close gripper") &&
      moveJoints(arm_group, logger, pre_grasp, "lift object");

  if (!success) {
    RCLCPP_ERROR(logger, "Grasp sequence aborted due to failure.");
    shutdown();
    return 1;
  }

  RCLCPP_INFO(logger, "Fixed joint-space grasp sequence finished successfully.");

  shutdown();
  return 0;
}