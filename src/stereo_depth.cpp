#include "stereo_depth.hpp"

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <opencv2/core.hpp>
#include <string>

// clang-format off
// todo: make this conditional at compile time
#define LTTNG_UST_TRACEPOINT_PROBE_DYNAMIC_LINKAGE
#include <slam_tracepoint_provider/tracepoint.hpp>
// clang-format on

namespace openvslam_ros {

StereoDepth::StereoDepth(std::shared_ptr<openvslam::config> const& cfg,
                         std::string const& vocab_file_path)
    : System(cfg, vocab_file_path),
      left_img_sub_(node_, "left/image_rect"),
      left_info_sub_(node_, "left/camera_info"),
      disp_sub_(node_, "disparity") {
  use_exact_time_ = true;
  use_exact_time_ = node_->declare_parameter("use_exact_time", use_exact_time_);
  if (use_exact_time_) {
    exact_time_sync_ = std::make_unique<ExactTimeSyncPolicy::Sync>(
        static_cast<uint32_t>(queue_size_), left_img_sub_, left_info_sub_,
        disp_sub_);
    exact_time_sync_->registerCallback(&StereoDepth::StereoDepthCallback, this);
  } else {
    approx_time_sync_ = std::make_unique<ApproximateTimeSyncPolicy::Sync>(
        static_cast<uint32_t>(queue_size_), left_img_sub_, left_info_sub_,
        disp_sub_);
    approx_time_sync_->registerCallback(&StereoDepth::StereoDepthCallback,
                                        this);
  }
}

void StereoDepth::StereoDepthCallback(
    ImageMsg::ConstSharedPtr const& img,
    CameraInfoMsg::ConstSharedPtr const& info,
    DisparityMsg::ConstSharedPtr const& disp) {
  TP_CALLBACK_ENTER(node_, node_->now(), __func__);
  auto leftcv = cv_bridge::toCvShare(img)->image;
  auto dispcv = cv_bridge::toCvShare(disp->image, disp)->image;

  const rclcpp::Time tp_1 = node_->now();
  // input the current frame and estimate the camera pose
  auto cam_pose_wc = GetSLAMSystem().feed_stereo_disparity_frame(
      leftcv, dispcv, rclcpp::Time(img->header.stamp).seconds());
  const rclcpp::Time tp_2 = node_->now();
  TP_COMPUTE_CPU(
      node_, std::chrono::nanoseconds(tp_2.nanoseconds() - tp_1.nanoseconds()),
      "slam:feed_stereo_disparity_frame");

  if (cam_pose_wc) {
    PublishPose(*cam_pose_wc, img->header.stamp);
  }
  TP_CALLBACK_EXIT(node_, node_->now(), __func__);
}

}  // namespace openvslam_ros