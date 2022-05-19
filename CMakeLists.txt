cmake_minimum_required(VERSION 3.5)
project(openvslam_ros LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)

if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW)
endif()
if(POLICY CMP0072)
    set(OpenGL_GL_PREFERENCE LEGACY)
endif()
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 OLD)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

##################
# build type #
##################
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")


#################
# Find packages #
#################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(slam_tracepoint_provider REQUIRED)
find_package(spdlog REQUIRED)

# thread library
find_package(Threads REQUIRED)

# OpenVSLAM
find_package(openvslam REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "Use OpenCV ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 3.3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# yaml-cpp
# find_package(yaml-cpp REQUIRED)

# popl
set(popl_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/popl/include)
include_directories(${popl_INCLUDE_DIR})

# pangolin viewer
find_package(Pangolin QUIET)
set(USE_PANGOLIN_VIEWER ${Pangolin_FOUND} CACHE BOOL "Enable Pangolin Viewer")
if(USE_PANGOLIN_VIEWER)
    find_library(Pangolin_Viewer_LIB pangolin_viewer)
    find_package(Pangolin REQUIRED)
endif()

#########
# Build #
#########

add_subdirectory(src)

ament_package()
