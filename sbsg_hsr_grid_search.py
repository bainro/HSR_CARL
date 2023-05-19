#!/bin/usr/python3
import os
import sys
import copy

if __name__ == "__main__":
  os.chdir(os.environ['HOME'])
  home_dir = os.getcwd()
  os.chdir('./Desktop')
  # make save_dir in ~/Desktop/sbsg_grid_search
  os.makedirs("sbsg_grid_search_2", exist_ok=True)
  os.chdir('./sbsg_grid_search_2')
  save_dir = os.getcwd()

  os.chdir(home_dir)
  # cd to cartographer configuration file location
  os.chdir("./cartographer_ws/src/cartographer_toyota_hsr/cartographer_toyota_hsr/configuration_files")
  cart_conf_dir = os.getcwd()
  base_lua = open("./sbsg_grid_search_base.lua", "r")
  base_lua = list(base_lua)
  
  # Hand-tuned seed also used for development
  '''
  POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
  POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e2
  POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e7 -- 1e5
  POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 1
  POSE_GRAPH.matcher_translation_weight = 5 -- 5e2
  '''
  # second search: min_score, global_localization_min_score, larger matcher_t_w & rotational weights
  '''
  POSE_GRAPH.matcher_translation_weight = 50
  POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e4
  POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
  POSE_GRAPH.constraint_builder.min_score = 0.55
  POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
  '''

  # 5-way grid search
  for w_pm_1 in ["50", "500"]:
    for w_pm_2 in ["1e3", "1e4", "1e5"]:
      for w_pm_3 in ["1e3", "1e5", "1e6"]:
        for w_pm_4 in ["0.65", "0.75"]:
          for w_pm_5 in ["0.6", "0.7"]:
            current_lua = copy.deepcopy(base_lua)
            current_lua.append("POSE_GRAPH.matcher_translation_weight = " + w_pm_1 + "\n")
            current_lua.append("POSE_GRAPH.optimization_problem.odometry_rotation_weight = " + w_pm_2 + "\n")
            current_lua.append("POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = " + w_pm_3 + "\n")
            current_lua.append("POSE_GRAPH.constraint_builder.min_score = " + w_pm_4 + "\n")
            current_lua.append("POSE_GRAPH.constraint_builder.global_localization_min_score = " + w_pm_5 + "\n")
            # append the next 5 custom pm lines
            current_lua.append("return options\n")
  
            # save file, overwriting previous "sbsg_grid_search.lua"
            os.chdir(cart_conf_dir)
            with open("./sbsg_grid_search.lua", 'w') as lf: 
              lf.writelines(current_lua)

            # stop any rosbags still playing & cartographer
            os.system("pkill rosbag")
            os.system("pkill cartographer")
            
            os.system("sleep 3")
            os.system("roslaunch cartographer_toyota_hsr carl_sbsg_grid.launch &")
            os.system("sleep 3")
            # Location of OG bag, had IMU data remapped to same frame as lidar
            # "/home/hsr/Downloads/SBSG_2nd_floor.bag"
            # weird glitch where 1st /tf messages aren't captured by catrographer
            for i_ in range(10):
              os.system("rosbag play --clock -u 0.2 --rate 0.5 /tmp/test.bag")
            os.system("rosbag play --clock -u 970 --rate 2.5 /tmp/test.bag")

            os.system("sleep 10")
            # cd back to save_dir
            os.chdir(save_dir)
            os.system("rosrun map_server map_saver --occ 49 --free 40")
            # filename based on current iteration's hyperparameters
            custom_file_name = str(w_pm_1 + "_" + w_pm_2 + "_" + w_pm_3 + "_" + w_pm_4 + "_" + w_pm_5 + ".pgm")
            os.system("cp map.pgm " + custom_file_name)