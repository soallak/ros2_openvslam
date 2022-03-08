#ifdef USE_PANGOLIN_VIEWER
#include <pangolin_viewer/viewer.h>
#endif

#include <openvslam/config.h>
#include <openvslam/system.h>
#include <openvslam/util/yaml.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <popl.hpp>

#include "system.hpp"

void tracking(const std::shared_ptr<openvslam::config>& cfg,
              const std::string& vocab_file_path,
              const std::string& mask_img_path, const bool eval_log,
              const std::string& map_db_path, const bool rectify) {
  // TODO: Replace this with a factormy method
  std::shared_ptr<openvslam_ros::system> ros = openvslam_ros::system::create(
      cfg, vocab_file_path, mask_img_path, rectify);

  auto& SLAM = ros->SLAM_;
  // startup the SLAM process
  SLAM.startup();

  // create a viewer object
  // and pass the frame_publisher and the map_publisher
#ifdef USE_PANGOLIN_VIEWER
  pangolin_viewer::viewer viewer(
      openvslam::util::yaml_optional_ref(cfg->yaml_node_, "PangolinViewer"),
      &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
#endif

  // TODO: Pangolin needs to run in the main thread on OSX
  // run the viewer in another thread
#ifdef USE_PANGOLIN_VIEWER
  std::thread thread([&]() {
    viewer.run();
    if (SLAM.terminate_is_requested()) {
      // wait until the loop BA is finished
      while (SLAM.loop_BA_is_running()) {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
      }
      rclcpp::shutdown();
    }
  });
#endif

  // TODO: this needs to change. The input rate must drives the output rate
  rclcpp::Rate rate(50);
  while (rclcpp::ok()) {
    ros->exec_.spin_some();
    rate.sleep();
  }

  // automatically close the viewer
#ifdef USE_PANGOLIN_VIEWER
  viewer.request_terminate();
  thread.join();
#endif

  // shutdown the SLAM process
  SLAM.shutdown();

  auto& track_times = ros->track_times_;
  if (eval_log) {
    // output the trajectories for evaluation
    SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
    SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
    // output the tracking times for evaluation
    std::ofstream ofs("track_times.txt", std::ios::out);
    if (ofs.is_open()) {
      for (const auto track_time : track_times) {
        ofs << track_time << std::endl;
      }
      ofs.close();
    }
  }

  if (!map_db_path.empty()) {
    // output the map database
    SLAM.save_map_database(map_db_path);
  }

  if (track_times.size()) {
    std::sort(track_times.begin(), track_times.end());
    const auto total_track_time =
        std::accumulate(track_times.begin(), track_times.end(), 0.0);
    RCLCPP_DEBUG(ros->node_->get_logger(), "Median tracking time: %f [s] ",
                 track_times.at(track_times.size() / 2));
    RCLCPP_DEBUG(ros->node_->get_logger(), "Mean tracking time: %f [s] ",
                 total_track_time / track_times.size());
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  // create options
  popl::OptionParser op("Allowed options");
  auto help = op.add<popl::Switch>("h", "help", "produce help message");
  auto vocab_file_path =
      op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
  auto setting_file_path =
      op.add<popl::Value<std::string>>("c", "config", "setting file path");
  auto mask_img_path =
      op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
  auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
  auto eval_log = op.add<popl::Switch>(
      "", "eval-log", "store trajectory and tracking times for evaluation");
  auto map_db_path = op.add<popl::Value<std::string>>(
      "", "map-db", "store a map database at this path after SLAM", "");
  auto rectify = op.add<popl::Switch>("r", "rectify", "rectify stereo image");
  try {
    op.parse(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    std::cerr << std::endl;
    std::cerr << op << std::endl;
    return EXIT_FAILURE;
  }

  // check validness of options
  if (help->is_set()) {
    std::cerr << op << std::endl;
    return EXIT_FAILURE;
  }
  if (!vocab_file_path->is_set() || !setting_file_path->is_set()) {
    std::cerr << "invalid arguments" << std::endl;
    std::cerr << std::endl;
    std::cerr << op << std::endl;
    return EXIT_FAILURE;
  }

  // setup logger
  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
  if (debug_mode->is_set()) {
    spdlog::set_level(spdlog::level::debug);
  } else {
    spdlog::set_level(spdlog::level::info);
  }

  // load configuration
  std::shared_ptr<openvslam::config> cfg;
  try {
    cfg = std::make_shared<openvslam::config>(setting_file_path->value());
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  // run tracking
  tracking(cfg, vocab_file_path->value(), mask_img_path->value(),
           eval_log->is_set(), map_db_path->value(), rectify->value());

  return EXIT_SUCCESS;
}
