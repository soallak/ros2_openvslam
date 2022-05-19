#include "stereo.hpp"

#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>

// clang-format off
// todo: make this conditional at compile time
#define LTTNG_UST_TRACEPOINT_DEFINE
#define LTTNG_UST_TRACEPOINT_PROBE_DYNAMIC_LINKAGE
#include <slam_tracepoint_provider/tracepoint.hpp>
// clang-format on

namespace openvslam_ros {

Stereo::Stereo(const std::shared_ptr<openvslam::config>& cfg,
               const std::string& vocab_file_path)
    : System(cfg, vocab_file_path),
      left_sf_(node_, "left/image_rect"),
      right_sf_(node_, "right/image_rect") {
  // Additional Parameters
  use_exact_time_ = false;
  use_exact_time_ = node_->declare_parameter("use_exact_time", use_exact_time_);

  if (use_exact_time_) {
    exact_time_sync_ = std::make_unique<ExactTimeSyncPolicy::Sync>(
        static_cast<uint32_t>(queue_size_), left_sf_, right_sf_);
    exact_time_sync_->registerCallback(&Stereo::StereoCallback, this);
  } else {
    approx_time_sync_ = std::make_unique<ApproximateTimeSyncPolicy::Sync>(
        10, left_sf_, right_sf_);
    approx_time_sync_->registerCallback(&Stereo::StereoCallback, this);
  }
}

void Stereo::StereoCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr& left,
    const sensor_msgs::msg::Image::ConstSharedPtr& right) {
  TP_CALLBACK_ENTER(node_, node_->now(), __func__);
  auto leftcv = cv_bridge::toCvShare(left)->image;
  auto rightcv = cv_bridge::toCvShare(right)->image;

  const rclcpp::Time tp_1 = node_->now();
  // input the current frame and estimate the camera pose

  auto cam_pose_wc = GetSLAMSystem().feed_stereo_frame(
      leftcv, rightcv, rclcpp::Time(left->header.stamp).seconds());

  const rclcpp::Time tp_2 = node_->now();
  TP_COMPUTE_CPU(
      node_, std::chrono::nanoseconds(tp_2.nanoseconds() - tp_1.nanoseconds()),
      "slam:feed_stereo_frame");

  if (cam_pose_wc) {
    PublishPose(*cam_pose_wc, left->header.stamp);
  }

  TP_CALLBACK_EXIT(node_, node_->now(), __func__);
}

}  // namespace openvslam_ros
