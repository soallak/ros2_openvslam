#ifndef OPENVSLAM_ROS_H
#define OPENVSLAM_ROS_H

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <openvslam/config.h>
#include <openvslam/system.h>
#include <openvslam/util/stereo_rectifier.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace openvslam_ros {
class system {
 public:
  static std::unique_ptr<system> create(
      std::shared_ptr<openvslam::config> const& cfg,
      std::string vocab_file_path, std::string maks_img_path);

  void publish_pose(const Eigen::Matrix4d& cam_pose_wc,
                    const rclcpp::Time& stamp);
  void setParams();
  openvslam::system SLAM_;
  std::shared_ptr<openvslam::config> cfg_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  rmw_qos_profile_t custom_qos_;
  cv::Mat mask_;
  std::vector<double> track_times_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pose_pub_;
  std::shared_ptr<
      rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>>
      init_pose_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> map_to_odom_broadcaster_;
  std::string map_frame_;
  std::string camera_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  bool publish_tf_;
  double transform_tolerance_;

 protected:
  system(const std::shared_ptr<openvslam::config>& cfg,
         const std::string& vocab_file_path, const std::string& mask_img_path);

 private:
  void init_pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
};

}  // namespace openvslam_ros

#endif  // OPENVSLAM_ROS_H
