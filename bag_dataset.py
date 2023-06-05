'''
Coverts a rosbag into a dataset for Perspective Transorming VAE(s)
'''
import os
import argparse
import rosbag

parser = argparse.ArgumentParser()
_help = "path to previously learned cartographer map (*.pbstream)"
parser.add_argument("map_file", type=str, required=True, help=_help)
_help = "path to previously recorded rosbag"
parser.add_argument("bag_file", type=str, required=True, help=_help)
args = parser.parse_args()

if __name__ == "__main__":
  # run a bag in offline localization-only mode (requires a previously learned SLAM map)
  os.system("rosparam use_sim_time true")
  # carl_localize.launch expects the map to be here
  os.system("cp " + args.map_file + " /tmp/current.pbstream")
  os.system("roslaunch cartographer_toyota_hsr carl_localize.launch &")
  
  # create copy of filtered /tf so they don't accumulate & conflict in the new bag
  os.system("rosbag filter " + args.bag_file + " /tmp/filtered.bag 'topic != \"/tf\"'")
  
  # weird glitch where earliest /tf messages aren't captured by catrographer
  for i in range(10):
    # need to play the OG, unfiltered one as we want to capture /tf msgs
    os.system("rosbag play --clock -u 0.2 --rate 0.2 " + args.bag_file)
  # save a new bag with robot's pose & the FPV camera images
  os.system("rosbag record /tf /image_proc_resize/image __name:=loc_bag &")
  os.system("rosbag play --clock --rate 2.5 /tmp/filtered.bag")
  os.system("rosnode kill /loc_bag")
  
  # load map.pgm from sys.argv
  # load rosbag from sys.argv
  bag = rosbag.Bag('/tmp/test.bag')
  for topic, msg, t in bag.read_messages(topics=['/tf', 'image_proc_resize/image']):
    print(msg)
  bag.close()
  # get a path of length X
  # overlay this path on the map 
  # use keys to translate & rotate the path
  # add buffer to map to enable rotate + crop
  # crop with robot position at center
  # rotate, then crop to final size
  # draw robot's pose (start simple, like an arrow)
  # restart and loop over the whole bag
  # save in the format Tim's already using (i.e. csv)
  # save cropped image of map & resized camera image
  
  # === FUTURE FEATURES ===
  # Include history channels (i.e. multiple stacked FPV images)
  # Generating GMP images using the blueprint. 
  # ^ Requires scaling offline or as additional script setting.
