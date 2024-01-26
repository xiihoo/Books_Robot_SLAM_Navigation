include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link", --default:"base_link","imu_link" if use imu
  published_frame = "base_link", --default:"base_link","odom" if use odometry
  odom_frame = "odom", --default:"odom",no-use if provide_odom_frame = false
  provide_odom_frame = true, --default:true
  publish_frame_projected_to_2d = false,
  use_odometry = false, --default:false
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1, --default:0
  num_multi_echo_laser_scans = 0, --default:1
  num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2, --default:0.2
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10 --default:10,72
--new params add
TRAJECTORY_BUILDER_2D.min_range = 0.20 --lidar:ydlidar-x4
TRAJECTORY_BUILDER_2D.max_range = 16.0 --lidar:ydlidar-x4
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40
POSE_GRAPH.constraint_builder.max_constraint_distance = 4.

return options
