#pragma once

#include "system.hpp"

namespace openvslam_ros {

class mono : public system {
 public:
  mono(const std::shared_ptr<openvslam::config>& cfg,
       const std::string& vocab_file_path, const std::string& mask_img_path);
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  image_transport::Subscriber sub_;
};

}  // namespace openvslam_ros
