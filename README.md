# openvslam_ros

[OpenVSLAM](https://github.com/OpenVSLAM-Community/openvslam)'s wrapper ROS2 package.

## Subscribed topics

### monocular setup

- `/initialpose`
- `camera/image_raw`

### stereo setup

- `/initialpose`
- `left/image_raw`
- `right/image_raw`

### RGBD setup

- `/initialpose`
- `camera/color/image_raw`
- `camera/depth/image_raw`

## Published topics

- `camera_pose`

## Parameters

- `map_frame`
- `camera_frame`
- `publish_tf`
- `transform_tolerance`
- `use_exact_time` (stereo, RGBD only)
