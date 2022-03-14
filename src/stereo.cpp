#include "stereo.hpp"

namespace openvslam_ros {

stereo::stereo(const std::shared_ptr<openvslam::config>& cfg,
               const std::string& vocab_file_path,
               const std::string& mask_img_path, const bool rectify)
    : system(cfg, vocab_file_path, mask_img_path),
      rectifier_(rectify
                     ? std::make_unique<openvslam::util::stereo_rectifier>(cfg)
                     : nullptr),
      left_sf_(node_, "left/image_rect"),
      right_sf_(node_, "right/image_rect") {
  use_exact_time_ = false;
  use_exact_time_ = node_->declare_parameter("use_exact_time", use_exact_time_);
  if (use_exact_time_) {
    exact_time_sync_ =
        std::make_unique<ExactTimeSyncPolicy::Sync>(2, left_sf_, right_sf_);
    exact_time_sync_->registerCallback(&stereo::callback, this);
  } else {
    approx_time_sync_ = std::make_unique<ApproximateTimeSyncPolicy::Sync>(
        10, left_sf_, right_sf_);
    approx_time_sync_->registerCallback(&stereo::callback, this);
  }
}

void stereo::callback(const sensor_msgs::msg::Image::ConstSharedPtr& left,
                      const sensor_msgs::msg::Image::ConstSharedPtr& right) {
  if (camera_optical_frame_.empty()) {
    camera_optical_frame_ = left->header.frame_id;
  }
  auto leftcv_color = cv_bridge::toCvShare(left)->image;
  auto rightcv_color = cv_bridge::toCvShare(right)->image;
  cv::Mat leftcv(leftcv_color.rows, leftcv_color.cols, CV_8UC1);
  cv::Mat rightcv(rightcv_color.rows, rightcv_color.cols, CV_8UC1);
  cvtColor(leftcv_color, leftcv, cv::COLOR_BGR2GRAY);
  cvtColor(rightcv_color, rightcv, cv::COLOR_BGR2GRAY);
  if (leftcv.empty() || rightcv.empty()) {
    return;
  }

  if (leftcv.rows != rightcv.rows || leftcv.cols != rightcv.cols) {
    // TODO(swallak) Improve this logging
    RCLCPP_ERROR_STREAM(node_->get_logger(),
                        "Leftsize: " << leftcv.rows << ", " << leftcv.cols
                                     << "-- Rightsize:" << rightcv.rows << ", "
                                     << rightcv.cols);
  }

  if (rectifier_) {
    rectifier_->rectify(leftcv, rightcv, leftcv, rightcv);
  }

  const rclcpp::Time tp_1 = node_->now();
  const double timestamp = tp_1.seconds();

  // input the current frame and estimate the camera pose
  cv::Mat mask;
  if (mask_.empty()) {
    mask = cv::Mat(leftcv.rows, leftcv.cols, CV_8UC1);
  } else {
    mask = mask_;
  }

  // RCLCPP_ERROR_STREAM(node_->get_logger(), "Leftsize: " << leftcv.rows << ",
  // " << leftcv.cols << "-- Rightsize:" << rightcv.rows << ", " << rightcv.cols
  // << "--Masksize: " << mask.rows << ", " << mask.cols);

  auto cam_pose_wc = SLAM_.feed_stereo_frame(leftcv, rightcv, timestamp, mask);

  const rclcpp::Time tp_2 = node_->now();
  const double track_time = (tp_2 - tp_1).seconds();

  // track times in seconds
  track_times_.push_back(track_time);

  if (cam_pose_wc) {
    publish_pose(*cam_pose_wc, left->header.stamp);
  }
}

}  // namespace openvslam_ros
