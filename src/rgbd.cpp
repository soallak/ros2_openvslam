#include "rgbd.hpp"

namespace openvslam_ros {

rgbd::rgbd(const std::shared_ptr<openvslam::config>& cfg,
           const std::string& vocab_file_path, const std::string& mask_img_path)
    : system(cfg, vocab_file_path, mask_img_path),
      color_sf_(node_, "camera/color/image_raw"),
      depth_sf_(node_, "camera/depth/image_raw") {
  use_exact_time_ = false;
  use_exact_time_ = node_->declare_parameter("use_exact_time", use_exact_time_);
  if (use_exact_time_) {
    exact_time_sync_ =
        std::make_unique<ExactTimeSyncPolicy::Sync>(2, color_sf_, depth_sf_);
    exact_time_sync_->registerCallback(&rgbd::callback, this);
  } else {
    approx_time_sync_ = std::make_unique<ApproximateTimeSyncPolicy::Sync>(
        10, color_sf_, depth_sf_);
    approx_time_sync_->registerCallback(&rgbd::callback, this);
  }
}

void rgbd::callback(const sensor_msgs::msg::Image::ConstSharedPtr& color,
                    const sensor_msgs::msg::Image::ConstSharedPtr& depth) {
  auto colorcv = cv_bridge::toCvShare(color)->image;
  auto depthcv = cv_bridge::toCvShare(depth)->image;
  if (colorcv.empty() || depthcv.empty()) {
    return;
  }
  if (depthcv.type() == CV_32FC1) {
    cv::patchNaNs(depthcv);
  }

  const rclcpp::Time tp_1 = node_->now();
  const double timestamp = tp_1.seconds();

  // input the current frame and estimate the camera pose
  auto cam_pose_wc = SLAM_.feed_RGBD_frame(colorcv, depthcv, timestamp, mask_);

  const rclcpp::Time tp_2 = node_->now();
  const double track_time = (tp_2 - tp_1).seconds();

  // track time in seconds
  track_times_.push_back(track_time);

  if (cam_pose_wc) {
    publish_pose(*cam_pose_wc, color->header.stamp);
  }
}

}  // namespace openvslam_ros