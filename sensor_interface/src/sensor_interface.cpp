#include "sensor_interface/sensor_interface.hpp"

namespace sensor_interface
{

SensorInterface::SensorInterface(const rclcpp::NodeOptions & options)
: Node("sensor_interface", options),
  publish_registered_topics_(true),
  publish_lidar_frame_cloud_(true),
  publish_base_odometry_(true),
  publish_base_tf_(true),
  base_frame_to_lidar_initialized_(false),
  has_previous_base_transform_(false)
{
  cloud_sub_topic_ = this->declare_parameter<std::string>("cloud_sub", "/cloud_registered");
  odometry_sub_topic_ = this->declare_parameter<std::string>("odometry_sub", "Odometry");
  registered_scan_topic_ =
    this->declare_parameter<std::string>("registered_scan_topic", "/registered_scan");
  registered_odometry_topic_ =
    this->declare_parameter<std::string>("registered_odometry_topic", "/registered_odometry");
  lidar_frame_pcd_topic_ =
    this->declare_parameter<std::string>("lidar_frame_pcd_topic", "/lidar_frame_pcd");
  base_odometry_topic_ = this->declare_parameter<std::string>("base_odometry_topic", "/odom");

  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  lidar_frame_ = this->declare_parameter<std::string>("lidar_frame", "livox_frame");
  base_footprint_frame_ =
    this->declare_parameter<std::string>("base_footprint_frame", "base_footprint");
  chassis_frame_ = this->declare_parameter<std::string>("chassis_frame", "chassis");

  publish_registered_topics_ =
    this->declare_parameter<bool>("publish_registered_topics", true);
  publish_lidar_frame_cloud_ =
    this->declare_parameter<bool>("publish_lidar_frame_cloud", true);
  publish_base_odometry_ =
    this->declare_parameter<bool>("publish_base_odometry", true);
  publish_base_tf_ = this->declare_parameter<bool>("publish_base_tf", true);

  tf_odom_to_lidar_odom_.setIdentity();
  previous_base_transform_.setIdentity();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  registered_scan_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(registered_scan_topic_, 5);
  registered_odometry_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>(registered_odometry_topic_, 5);
  lidar_frame_pcd_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_frame_pcd_topic_, 2);
  base_odometry_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>(base_odometry_topic_, 2);

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, odometry_sub_topic_, qos_profile);
  cloud_sub_.subscribe(this, cloud_sub_topic_, qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, cloud_sub_);
  sync_->registerCallback(
    std::bind(
      &SensorInterface::sensorDataCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), MAG "SensorInterface node is START" RST);
}

SensorInterface::~SensorInterface()
{
  RCLCPP_INFO(this->get_logger(), MAG "SensorInterface node is OVER" RST);
}

void SensorInterface::sensorDataCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg)
{
  if (!initializeBaseToLidarTransform()) {
    return;
  }

  sensor_msgs::msg::PointCloud2 registered_scan;
  pcl_ros::transformPointCloud(
    odom_frame_, tf_odom_to_lidar_odom_, *cloud_msg, registered_scan);
  if (publish_registered_topics_) {
    registered_scan_pub_->publish(registered_scan);
  }

  tf2::Transform tf_lidar_odom_to_lidar;
  tf2::fromMsg(odometry_msg->pose.pose, tf_lidar_odom_to_lidar);
  const tf2::Transform tf_odom_to_lidar =
    tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar;

  const auto registered_odometry =
    createRegisteredOdometry(tf_odom_to_lidar, odometry_msg->header.stamp);
  if (publish_registered_topics_) {
    registered_odometry_pub_->publish(registered_odometry);
  }

  const auto tf_lidar_to_base_footprint =
    getTransform(lidar_frame_, base_footprint_frame_, cloud_msg->header.stamp);
  const auto tf_lidar_to_chassis =
    getTransform(lidar_frame_, chassis_frame_, cloud_msg->header.stamp);
  const auto tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;
  (void)tf_odom_to_chassis;

  const auto tf_odom_to_base_footprint =
    tf_odom_to_lidar * tf_lidar_to_base_footprint;

  if (publish_base_tf_) {
    publishTransform(
      tf_odom_to_base_footprint, registered_odometry.header.frame_id,
      base_footprint_frame_, cloud_msg->header.stamp);
  }

  if (publish_base_odometry_) {
    publishBaseOdometry(
      tf_odom_to_base_footprint, registered_odometry.header.frame_id,
      base_footprint_frame_, cloud_msg->header.stamp);
  }

  if (publish_lidar_frame_cloud_) {
    sensor_msgs::msg::PointCloud2 lidar_frame_cloud;
    pcl_ros::transformPointCloud(
      lidar_frame_, tf_odom_to_lidar.inverse(), registered_scan, lidar_frame_cloud);
    lidar_frame_pcd_pub_->publish(lidar_frame_cloud);
  }
}

bool SensorInterface::initializeBaseToLidarTransform()
{
  if (base_frame_to_lidar_initialized_) {
    return true;
  }

  try {
    const auto transform_stamped = tf_buffer_->lookupTransform(
      base_footprint_frame_, lidar_frame_, tf2::TimePointZero, std::chrono::seconds(1));

    tf2::Transform tf_base_frame_to_lidar_frame;
    tf2::fromMsg(transform_stamped.transform, tf_base_frame_to_lidar_frame);
    tf_odom_to_lidar_odom_ = tf_base_frame_to_lidar_frame;
    base_frame_to_lidar_initialized_ = true;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
    return false;
  }
}

nav_msgs::msg::Odometry SensorInterface::createRegisteredOdometry(
  const tf2::Transform & transform, const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = lidar_frame_;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());
  return out;
}

tf2::Transform SensorInterface::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const rclcpp::Time & time)
{
  (void)time;

  try {
    const auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
    tf2::Transform transform;
    tf2::fromMsg(transform_stamped.transform, transform);
    return transform;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Returning identity.", ex.what());
    return tf2::Transform::getIdentity();
  }
}

void SensorInterface::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  tf_broadcaster_->sendTransform(transform_msg);
}

void SensorInterface::publishBaseOdometry(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  if (has_previous_base_transform_) {
    const double dt = (stamp - previous_base_stamp_).seconds();
    if (dt > 0.0) {
      const auto linear_velocity =
        (transform.getOrigin() - previous_base_transform_.getOrigin()) / dt;

      const tf2::Quaternion q_diff =
        transform.getRotation() * previous_base_transform_.getRotation().inverse();
      const auto angular_velocity = q_diff.getAxis() * q_diff.getAngle() / dt;

      out.twist.twist.linear.x = linear_velocity.x();
      out.twist.twist.linear.y = linear_velocity.y();
      out.twist.twist.linear.z = linear_velocity.z();
      out.twist.twist.angular.x = angular_velocity.x();
      out.twist.twist.angular.y = angular_velocity.y();
      out.twist.twist.angular.z = angular_velocity.z();
    }
  }

  previous_base_transform_ = transform;
  previous_base_stamp_ = stamp;
  has_previous_base_transform_ = true;

  base_odometry_pub_->publish(out);
}

}  // namespace sensor_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_interface::SensorInterface)
