#include "system.hpp"

#include <geometry_msgs/msg/transform_stamped.h>
#include <openvslam/publish/map_publisher.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "stereo.hpp"

namespace openvslam_ros {

std::unique_ptr<system> system::create(
    std::shared_ptr<openvslam::config> const& cfg, std::string vocab_file_path,
    std::string mask_img_path) {
  if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo) {
    return std::make_unique<openvslam_ros::stereo>(cfg, vocab_file_path,
                                                   mask_img_path);
  } else {
    throw std::runtime_error("Invalid setup type: " +
                             cfg->camera_->get_setup_type_string());
  }
}

system::system(const std::shared_ptr<openvslam::config>& cfg,
               const std::string& vocab_file_path,
               const std::string& mask_img_path)
    : SLAM_(cfg, vocab_file_path),
      cfg_(cfg),
      node_(std::make_unique<rclcpp::Node>("slam")),
      custom_qos_(rmw_qos_profile_default),
      mask_(mask_img_path.empty()
                ? cv::Mat{}
                : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE)),
      pose_pub_(
          node_->create_publisher<nav_msgs::msg::Odometry>("camera_pose", 1)),
      map_to_odom_broadcaster_(
          std::make_unique<tf2_ros::TransformBroadcaster>(node_)),
      tf_(std::make_unique<tf2_ros::Buffer>(node_->get_clock())),
      transform_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_)) {
  custom_qos_.depth = 1;
  exec_.add_node(node_);
  init_pose_sub_ =
      node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 1,
          std::bind(&system::init_pose_callback, this, std::placeholders::_1));
  setParams();
}

void system::publish_pose(const Eigen::Matrix4d& cam_pose_wc,
                          const rclcpp::Time& stamp) {
  // Extract rotation matrix and translation vector from
  Eigen::Matrix3d rot(cam_pose_wc.block<3, 3>(0, 0));
  Eigen::Translation3d trans(cam_pose_wc.block<3, 1>(0, 3));
  Eigen::Affine3d map_to_camera_affine(trans * rot);
  Eigen::AngleAxisd rot_ros_to_cv_map_frame(
      (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished());

  // Transform map frame from CV coordinate system to ROS coordinate system
  map_to_camera_affine.prerotate(rot_ros_to_cv_map_frame);

  // Create odometry message and update it with current camera pose
  nav_msgs::msg::Odometry pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = map_frame_;
  pose_msg.child_frame_id = camera_frame_;
  pose_msg.pose.pose =
      tf2::toMsg(map_to_camera_affine * rot_ros_to_cv_map_frame.inverse());
  pose_pub_->publish(pose_msg);

  // Send map->odom transform. Set publish_tf to false if not using TF
  if (publish_tf_) {
    try {
      auto map_to_camera_msg = tf2::eigenToTransform(map_to_camera_affine);
      tf2::TimePoint transform_timestamp =
          tf2_ros::fromMsg(stamp) + tf2::durationFromSec(transform_tolerance_);
      map_to_camera_msg.header.stamp = tf2_ros::toMsg(transform_timestamp);
      map_to_camera_msg.header.frame_id = map_frame_;
      map_to_camera_msg.child_frame_id = camera_frame_;
      map_to_odom_broadcaster_->sendTransform(map_to_camera_msg);
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(),
                                   1000, "Transform failed: " << ex.what());
    }
  }
}

void system::setParams() {
  map_frame_ = std::string("map_frame");
  map_frame_ = node_->declare_parameter("map_frame", map_frame_);

  camera_frame_ = std::string("camera_frame");
  camera_frame_ = node_->declare_parameter("camera_frame", camera_frame_);

  // Set publish_tf to false if not using TF
  publish_tf_ = true;
  publish_tf_ = node_->declare_parameter("publish_tf", publish_tf_);

  // Publish pose's timestamp in the future
  transform_tolerance_ = 0.5;
  transform_tolerance_ =
      node_->declare_parameter("transform_tolerance", transform_tolerance_);
}

void system::init_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  Eigen::Translation3d trans(msg->pose.pose.position.x,
                             msg->pose.pose.position.y,
                             msg->pose.pose.position.z);
  Eigen::Quaterniond rot_q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  Eigen::Affine3d initialpose_affine(trans * rot_q);

  Eigen::Matrix3d rot_cv_to_ros_map_frame;
  rot_cv_to_ros_map_frame << 0, -1, 0, 0, 0, -1, 1, 0, 0;

  Eigen::Affine3d map_to_initialpose_frame_affine;
  try {
    auto map_to_initialpose_frame = tf_->lookupTransform(
        map_frame_, msg->header.frame_id, tf2_ros::fromMsg(msg->header.stamp),
        tf2::durationFromSec(0.0));
    map_to_initialpose_frame_affine =
        tf2::transformToEigen(map_to_initialpose_frame.transform);
  } catch (tf2::TransformException& ex) {
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "Transform failed: " << ex.what());
    return;
  }

  Eigen::Matrix4d cam_pose_cv =
      (rot_cv_to_ros_map_frame * map_to_initialpose_frame_affine *
       initialpose_affine)
          .matrix();

  const Eigen::Vector3d normal_vector =
      (Eigen::Vector3d() << 0., 1., 0.).finished();
  if (!SLAM_.relocalize_by_pose_2d(cam_pose_cv, normal_vector)) {
    RCLCPP_ERROR(node_->get_logger(), "Can not set initial pose");
  }
}

}  // namespace openvslam_ros
