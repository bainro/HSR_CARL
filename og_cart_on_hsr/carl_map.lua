-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.1,
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

TRAJECTORY_BUILDER_2D.min_range = 0.
TRAJECTORY_BUILDER_2D.max_range = 40. -- 20.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

POSE_GRAPH.optimization_problem.huber_scale = 1e2
--POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e7
POSE_GRAPH.max_num_final_iterations = 2000

POSE_GRAPH.constraint_builder.sampling_ratio = 0.05 -- 0.3
POSE_GRAPH.matcher_rotation_weight = 1.6e2 -- 1.6e3
POSE_GRAPH.global_sampling_ratio = 0.0003 -- 0.003
POSE_GRAPH.optimize_every_n_nodes = 20 -- 90

-- found from grid search
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e2
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e6
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 1
POSE_GRAPH.matcher_translation_weight = 500
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e2 -- 1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e2 -- 1e3
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
return options