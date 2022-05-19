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

void Tracking(const std::shared_ptr<openvslam::config>& cfg,
              const std::string& vocab_file_path, const bool eval_log,
              const std::string& map_db_path) {
  std::shared_ptr<openvslam_ros::System> ros =
      openvslam_ros::System::Create(cfg, vocab_file_path);

  ros->Start();

  auto& SLAM = ros->GetSLAMSystem();
  if (eval_log) {
    // output the trajectories for evaluation
    SLAM.save_frame_trajectory("frame_trajectory.txt", "TUM");
    SLAM.save_keyframe_trajectory("keyframe_trajectory.txt", "TUM");
  }
  if (!map_db_path.empty()) {
    // output the map database
    SLAM.save_map_database(map_db_path);
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

  auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
  auto eval_log = op.add<popl::Switch>(
      "", "eval-log", "store trajectory and tracking times for evaluation");
  auto map_db_path = op.add<popl::Value<std::string>>(
      "", "map-db", "store a map database at this path after SLAM", "");

  try {
    op.parse(argc, argv);
  } catch (const std::exception& e) {
    spdlog::error(fmt::format("{}", e.what()));
    spdlog::error(fmt::format("{}", op.description()));
    return EXIT_FAILURE;
  }

  // check validness of options
  if (help->is_set()) {
    std::cerr << op << std::endl;
    return EXIT_FAILURE;
  }
  if (!vocab_file_path->is_set() || !setting_file_path->is_set()) {
    spdlog::error("Invalid arguments");
    spdlog::error(fmt::format("{}", op.description()));
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
  Tracking(cfg, vocab_file_path->value(), eval_log->is_set(),
           map_db_path->value());

  return EXIT_SUCCESS;
}
