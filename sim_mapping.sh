#!/bin/bash

# 加载 bashrc
source ~/.bashrc

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
SETUP_FILE="$WORKSPACE_ROOT/install/setup.bash"

if [ ! -f "$SETUP_FILE" ]; then
  echo "未找到 ROS 2 工作空间环境文件：$SETUP_FILE"
  echo "请先在工作空间中执行 colcon build"
  exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "未找到 gnome-terminal，请先安装 gnome-terminal"
  exit 1
fi

cd "$WORKSPACE_ROOT" || exit 1

terminal_args=()

add_window() {
  local title="$1"
  local delay="$2"
  local command="$3"

  terminal_args+=(
    --window
    --title="$title"
    --working-directory="$WORKSPACE_ROOT"
    -- bash -c "
      source \"$SETUP_FILE\";
      cd \"$WORKSPACE_ROOT\";
      sleep $delay;
      $command;
      exec bash"
  )
}

# 启动第一个launch文件：Gazebo 仿真
add_window "Gazebo 仿真" 0 "
  killall -9 gzserver gzclient 2>/dev/null || true;
  source /usr/share/gazebo/setup.bash;
  ros2 launch mini_cobot_moveit_config gazebo_moveit.launch.py"

# 启动第二个launch文件：FAST-LIO 里程计
add_window "FAST-LIO 里程计" 1 \
  "ros2 launch fast_lio mapping.launch.py"

# 启动第三个launch文件：LIO 接口
add_window "LIO 接口" 2 \
  "ros2 launch lio_interface lio_interface_launch.py"

# 启动第四个launch文件：传感器扫描生成
add_window "传感器扫描生成" 3 \
  "ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

# 启动第五个launch文件：3D 点云转 2D 激光
add_window "3D 点云转 2D 激光" 4 \
  "ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py"

# 启动第六个launch文件：KISS + GICP 重定位
add_window "KISS + GICP 重定位" 5 \
  "ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch.py"

# 启动第七个launch文件：Nav2 导航
add_window "Nav2 导航" 6 \
  "ros2 launch scout_bringup scout_nav2_bringup.launch.py"

gnome-terminal "${terminal_args[@]}"

echo "所有 launch 文件启动指令已提交。"
