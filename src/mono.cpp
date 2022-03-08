#include "mono.hpp"

namespace openvslam_ros {

mono::mono(const std::shared_ptr<openvslam::config>& cfg,
           const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path) {
  sub_ = image_transport::create_subscription(
      node_.get(), "camera/image_raw",
      [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        callback(msg);
      },
      "raw", custom_qos_);
}
void mono::callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  if (camera_optical_frame_.empty()) {
    camera_optical_frame_ = msg->header.frame_id;
  }
  const rclcpp::Time tp_1 = node_->now();
  const double timestamp = tp_1.seconds();

  // input the current frame and estimate the camera pose
  auto cam_pose_wc = SLAM_.feed_monocular_frame(
      cv_bridge::toCvShare(msg)->image, timestamp, mask_);

  const rclcpp::Time tp_2 = node_->now();
  const double track_time = (tp_2 - tp_1).seconds();

  // track times in seconds
  track_times_.push_back(track_time);

  if (cam_pose_wc) {
    publish_pose(*cam_pose_wc, msg->header.stamp);
  }
}

}  // namespace openvslam_ros