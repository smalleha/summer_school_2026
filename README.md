# Mini cobot 仿真

## 1. 环境要求

- **操作系统**：Ubuntu 22.04
- **ROS 2**：Humble Hawksbill
- **Gazebo**：Fortress

## 2、编译代码

```
mkdir -p agilex_ws/src
cd agilex_ws/src
git clone https://github.com/smalleha/summer_school_2026.git
cd ..
colcon build --symlink-install
```

## 3、快速开始

启动仿真环境

```
ros2 launch mini_cobot_moveit_config gazebo_moveit.launch.py
```

启动FAST-LIO 里程计

```
ros2 launch fast_lio mapping.launch.py
```

启动3D点云转2D激光

```
ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py
```

启动数据转换代码

```
ros2 launch lio_interface lio_interface_launch.py

ros2 launch sensor_scan_generation sensor_scan_generation_launch.py
```

启动SLAM Toolbox

```
ros2 launch scout_bringup slam_toolbox.launch.py
```

启动nav2导航

```
ros2 launch scout_bringup scout_nav2_bringup.launch.py
```

保存3D地图

```
ros2 service call /map_save std_srvs/srv/Trigger
```

保存2D地图

```
ros2 run nav2_map_server map_saver_cli -f map111
```















