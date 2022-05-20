# openvslam_ros

[OpenVSLAM](https://github.com/OpenVSLAM-Community/openvslam)'s wrapper ROS2 package.

## Subscribed topics

### Stereo setup

- `/initialpose`
- `left/image_raw`
- `right/image_raw`

### Stereo-Depth setup

- `/initialpose`
- `left/image_rect`
- `left/camera_info`
- `disparity`

## Published topics

- `camera_pose`

## Parameters

- `map_frame`
- `camera_frame`
- `publish_tf`
- `transform_tolerance`
- `use_exact_time`
- `queue_size`
- `start_pangolin_viewer`
