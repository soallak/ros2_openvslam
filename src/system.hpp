#pragma once

#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#endif

#include <openvslam/config.h>
#include <openvslam/system.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

namespace openvslam_ros {
class System {
 public:
  enum class Type { Stereo = 0, StereoDepth = 1 };

  static std::unique_ptr<System> Create(
      Type type, std::shared_ptr<openvslam::config> const& cfg,
      std::string vocab_file_path);

  virtual ~System();
  [[deprecated]] openvslam::system& GetSLAMSystem();

  void Start();

  void Stop();

 protected:
  System(const std::shared_ptr<openvslam::config>& cfg,
         const std::string& vocab_file_path);

  void PublishPose(Eigen::Matrix4d const& cam_pose_wc,
                   rclcpp::Time const& stamp);

 protected:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> pose_pub_;
  rclcpp::executors::SingleThreadedExecutor exec_;
  int queue_size_ = 10;

 private:
  void DeclareAndSetParams();

  void InitPoseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

 private:
  // tf2 related
  std::shared_ptr<
      rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>>
      init_pose_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> map_to_odom_broadcaster_;
  std::string map_frame_;
  std::string camera_frame_;
  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  double transform_tolerance_;
  bool publish_tf_;

  // SLAM
  openvslam::system slam_;
  std::shared_ptr<openvslam::config> cfg_;

  bool is_running_ = false;

// Pangolin viewer
#ifdef USE_PANGOLIN_VIEWER
  std::unique_ptr<pangolin_viewer::viewer> pangolin_viewer_;
  std::thread pangolin_viewer_thread_;
  bool start_pangolin_viewer_ = false;
#endif
};

}  // namespace openvslam_ros
