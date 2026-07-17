#ifndef SENSOR_INTERFACE_SENSOR_INTERFACE_HPP_
#define SENSOR_INTERFACE_SENSOR_INTERFACE_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#define RST "\033[0m"
#define BLU "\033[34m"
#define MAG "\033[35m"

namespace sensor_interface
{

class SensorInterface : public rclcpp::Node
{
public:
  explicit SensorInterface(const rclcpp::NodeOptions & options);
  ~SensorInterface() override;

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

  void sensorDataCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);

  bool initializeBaseToLidarTransform();

  nav_msgs::msg::Odometry createRegisteredOdometry(
    const tf2::Transform & transform, const rclcpp::Time & stamp) const;

  tf2::Transform getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & time);

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  void publishBaseOdometry(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  std::string cloud_sub_topic_;
  std::string odometry_sub_topic_;
  std::string registered_scan_topic_;
  std::string registered_odometry_topic_;
  std::string lidar_frame_pcd_topic_;
  std::string base_odometry_topic_;
  std::string odom_frame_;
  std::string lidar_frame_;
  std::string base_footprint_frame_;
  std::string chassis_frame_;

  bool publish_registered_topics_;
  bool publish_lidar_frame_cloud_;
  bool publish_base_odometry_;
  bool publish_base_tf_;
  bool base_frame_to_lidar_initialized_;
  bool has_previous_base_transform_;

  tf2::Transform tf_odom_to_lidar_odom_;
  tf2::Transform previous_base_transform_;
  rclcpp::Time previous_base_stamp_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr registered_odometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_frame_pcd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_odometry_pub_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

}  // namespace sensor_interface

#endif  // SENSOR_INTERFACE_SENSOR_INTERFACE_HPP_
