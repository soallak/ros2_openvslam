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

std::unique_ptr<System> System::Create(
    std::shared_ptr<openvslam::config> const& cfg,
    std::string vocab_file_path) {
  if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Stereo) {
    return std::make_unique<openvslam_ros::Stereo>(cfg, vocab_file_path);
  } else {
    throw std::runtime_error("Invalid setup type: " +
                             cfg->camera_->get_setup_type_string());
  }
}

System::System(const std::shared_ptr<openvslam::config>& cfg,
               const std::string& vocab_file_path)
    : slam_(cfg, vocab_file_path),
      cfg_(cfg),
      node_(std::make_unique<rclcpp::Node>("slam")),

      map_to_odom_broadcaster_(
          std::make_unique<tf2_ros::TransformBroadcaster>(node_)),
      tf_(std::make_unique<tf2_ros::Buffer>(node_->get_clock())),
      transform_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_)) {
  DeclareAndSetParams();
  pose_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("camera_pose",
                                                               queue_size_);
  init_pose_sub_ =
      node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", queue_size_,
          std::bind(&System::InitPoseCallback, this, std::placeholders::_1));
}  // namespace openvslam_ros

openvslam::system& System::GetSLAMSystem() { return slam_; }

System::~System() { Stop(); }

void System::Start() {
  if (is_running_) return;
  exec_.add_node(node_);
#ifdef USE_PANGOLIN_VIEWER
  if (start_pangolin_viewer_) {
    pangolin_viewer_ = std::make_unique<pangolin_viewer::viewer>(
        openvslam::util::yaml_optional_ref(cfg_->yaml_node_, "PangolinViewer"),
        &slam_, slam_.get_frame_publisher(), slam_.get_map_publisher());

    pangolin_viewer_thread_ = std::thread([&]() {
      pangolin_viewer_->run();
      if (slam_.terminate_is_requested()) {
        // wait until the loop BA is finished
        while (slam_.loop_BA_is_running()) {
          std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
      }
    });
  }
#endif
  slam_.startup();
  is_running_ = true;
  exec_.spin();
}

void System::Stop() {
  if (!is_running_) return;
  is_running_ = false;
#ifdef USE_PANGOLIN_VIEWER
  if (pangolin_viewer_) {
    pangolin_viewer_->request_terminate();
  }
#endif
  slam_.shutdown();
  exec_.cancel();
#ifdef USE_PANGOLIN_VIEWER
  if (pangolin_viewer_thread_.joinable()) {
    pangolin_viewer_thread_.join();
  }
#endif
}

void System::PublishPose(const Eigen::Matrix4d& cam_pose_wc,
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

void System::DeclareAndSetParams() {
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

  queue_size_ = 10;
  node_->declare_parameter("queue_size", queue_size_);

#ifdef USE_PANGOLIN_VIEWER
  start_pangolin_viewer_ = true;
  node_->declare_parameter("start_pangolin_viewer", start_pangolin_viewer_);
#endif
}

void System::InitPoseCallback(
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
  if (!slam_.relocalize_by_pose_2d(cam_pose_cv, normal_vector)) {
    RCLCPP_ERROR(node_->get_logger(), "Can not set initial pose");
  }
}

}  // namespace openvslam_ros
