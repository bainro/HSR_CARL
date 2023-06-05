'''
Coverts a rosbag into a dataset for Perspective Transorming VAE(s)
'''
import os
import argparse
import rosbag

parser = argparse.ArgumentParser()
_help = "path to previously learned cartographer map (*.pbstream)"
parser.add_argument("map_file", type=str, required=True, help=_help)
args = parser.parse_args()

if __name__ == "__main__":
  # run a bag in offline localization-only mode (requires a previously learned SLAM map)
  # carl_localize.launch expects the map to be here
  os.system("cp " + args.map_file + " /tmp/current.pbstream")
  os.system("roslaunch cartographer_toyota_hsr carl_localize.launch &")
  
  # save a new bag with robot's pose & the FPV camera images
  os.system("rosbag record /tf /image_proc_resize/image")
  
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