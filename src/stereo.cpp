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

stereo::stereo(const std::shared_ptr<openvslam::config>& cfg,
               const std::string& vocab_file_path,
               const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path),
      left_sf_(node_, "left/image_rect"),
      right_sf_(node_, "right/image_rect") {
  use_exact_time_ = false;
  use_exact_time_ = node_->declare_parameter("use_exact_time", use_exact_time_);
  if (use_exact_time_) {
    exact_time_sync_ =
        std::make_unique<ExactTimeSyncPolicy::Sync>(10, left_sf_, right_sf_);
    exact_time_sync_->registerCallback(&stereo::callback, this);
  } else {
    approx_time_sync_ = std::make_unique<ApproximateTimeSyncPolicy::Sync>(
        10, left_sf_, right_sf_);
    approx_time_sync_->registerCallback(&stereo::callback, this);
  }
}

void stereo::callback(const sensor_msgs::msg::Image::ConstSharedPtr& left,
                      const sensor_msgs::msg::Image::ConstSharedPtr& right) {
  TP_CALLBACK_ENTER(node_, node_->now(), __func__);
  auto leftcv = cv_bridge::toCvShare(left)->image;
  auto rightcv = cv_bridge::toCvShare(right)->image;

  const rclcpp::Time tp_1 = node_->now();
  // input the current frame and estimate the camera pose

  auto cam_pose_wc = SLAM_.feed_stereo_frame(
      leftcv, rightcv, rclcpp::Time(left->header.stamp).seconds());

  const rclcpp::Time tp_2 = node_->now();
  TP_COMPUTE_CPU(
      node_, std::chrono::nanoseconds(tp_2.nanoseconds() - tp_1.nanoseconds()),
      std::string(__func__) + ":feed_stereo_frame");
  const double track_time = (tp_2 - tp_1).seconds();

  // track times in seconds
  track_times_.push_back(track_time);

  if (cam_pose_wc) {
    publish_pose(*cam_pose_wc, left->header.stamp);
  }

  TP_CALLBACK_ENTER(node_, node_->now(), __func__);
}

}  // namespace openvslam_ros
