#pragma once
#include "system.hpp"

namespace openvslam_ros {

class stereo : public system {
 public:
  stereo(const std::shared_ptr<openvslam::config>& cfg,
         const std::string& vocab_file_path, const std::string& mask_img_path);

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr& left,
                const sensor_msgs::msg::Image::ConstSharedPtr& right);

  message_filters::Subscriber<sensor_msgs::msg::Image> left_sf_, right_sf_,
      right_rect_sf_, left_rect_sf_;
  using ApproximateTimeSyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image>;
  std::shared_ptr<ApproximateTimeSyncPolicy::Sync> approx_time_sync_;
  using ExactTimeSyncPolicy =
      message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                sensor_msgs::msg::Image>;
  std::shared_ptr<ExactTimeSyncPolicy::Sync> exact_time_sync_;
  bool use_exact_time_;
};

}  // namespace openvslam_ros