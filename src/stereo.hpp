#pragma once
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <openvslam/config.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "system.hpp"

namespace openvslam_ros {

class Stereo : public System {
 public:
  Stereo(const std::shared_ptr<openvslam::config>& cfg,
         const std::string& vocab_file_path);

  ~Stereo() = default;

 private:
  void StereoCallback(const sensor_msgs::msg::Image::ConstSharedPtr& left,
                      const sensor_msgs::msg::Image::ConstSharedPtr& right);

  message_filters::Subscriber<sensor_msgs::msg::Image> left_sf_, right_sf_;
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