#pragma once
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <openvslam/config.h>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>

#include "system.hpp"

namespace openvslam_ros {

class StereoDepth : public System {
 public:
  StereoDepth(std::shared_ptr<openvslam::config> const& cfg,
              std::string const& vocab_file_path);

  ~StereoDepth() = default;

 private:
  using ImageMsg = sensor_msgs::msg::Image;
  using DisparityMsg = stereo_msgs::msg::DisparityImage;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;

  void StereoDepthCallback(ImageMsg::ConstSharedPtr const& img,
                           CameraInfoMsg::ConstSharedPtr const& info,
                           DisparityMsg::ConstSharedPtr const& disp);

  message_filters::Subscriber<sensor_msgs::msg::Image> left_img_sub_;
  message_filters::Subscriber<stereo_msgs::msg::DisparityImage> disp_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_info_sub_;

  using ApproximateTimeSyncPolicy =
      message_filters::sync_policies::ApproximateTime<ImageMsg, CameraInfoMsg,
                                                      DisparityMsg>;

  using ExactTimeSyncPolicy =
      message_filters::sync_policies::ExactTime<ImageMsg, CameraInfoMsg,
                                                DisparityMsg>;

  std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
  std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
  bool use_exact_time_ = false;
};
}  // namespace openvslam_ros