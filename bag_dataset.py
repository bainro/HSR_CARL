'''
Coverts a rosbag into a dataset for Perspective Transorming VAE(s)
'''
import os
import time
import math
import rosbag
import argparse
import matplotlib.pyplot as plt
from pynput import keyboard as kb

parser = argparse.ArgumentParser()
_help = "path to previously learned cartographer map (*.pbstream)"
parser.add_argument("--map_file", type=str, required=True, help=_help)
_help = "path to previously recorded rosbag"
parser.add_argument("--bag_file", type=str, required=True, help=_help)
args = parser.parse_args()
assert args.map_file != "" and args.bag_file != "", "Must specify path to *.pbstream & *.bag files!"

if __name__ == "__main__":
  # run a bag in offline localization-only mode (requires a previously learned SLAM map)
  os.system("rosparam set use_sim_time true")
  # create copy of filtered /tf so they don't accumulate & conflict in the new bag
  os.system("rosbag filter " + args.bag_file + " /tmp/filtered.bag 'topic != \"/tf\"'")
  # carl_localize.launch expects the map to be here
  os.system("cp " + args.map_file + " /tmp/current.pbstream")
  os.system("roslaunch cartographer_toyota_hsr carl_localize.launch &")
  os.system("rosrun map_server map_saver --occ 49 --free 40 -f '/tmp/map'")
  
  # weird glitch where earliest /tf messages aren't captured by catrographer
  for i in range(10):
    # need to play the OG, unfiltered one as we want to capture /tf msgs
    os.system("rosbag play --clock -u 0.2 --rate 0.2 " + args.bag_file)
  # save a new bag with robot's pose & the FPV camera images
  os.system("rosbag record -O '/tmp/loc.bag' /tf /image_proc_resize/image __name:=loc_bag &")
  os.system("rosbag play --clock --rate 2.5 /tmp/filtered.bag")
  os.system("pkill cart")
  os.system("rosnode kill /loc_bag")
  
  time.sleep(1)
  # load rosbag & get a path of length non-significant length
  bag = rosbag.Bag('/tmp/loc.bag')
  path_x, path_y = [], []
  for topic, msg, t in bag.read_messages(topics=['/tf']):
    frame_id = msg.transforms[0].header.frame_id
    child_id = msg.transforms[0].child_frame_id
    # robot's pose inferred from transformations between the global map & odom frame
    if frame_id != "map" and child_id != "odom":
        continue
    path_x.append(msg.transforms[0].transform.translation.x)
    path_y.append(msg.transforms[0].transform.translation.y)
    # triangle maths
    path_dist = ((path_x[0] - path_x[-1]) ** 2 + (path_y[0] - path_y[-1]) ** 2) ** 0.5
    if path_dist > 10 or len(path_x) > 10:
      print("@TODO remove 2nd conditional for dbg")
      break
  bag.close()
  
  # ensure path points start as positive values
  min_x = min(path_x)
  if min_x < 0:
    path_x = [x - min_x for x in path_x]
  min_y = min(path_y)
  if min_y < 0:
    path_y = [y - min_y for y in path_y]
  
  # use keys to translate, rotate, & scale the path
  rot = 0 # rotation factor in radians
  scale = 1
  shift_on = False
  enter_pressed = False
  
  def up_cb(shift):
    global scale, path_y
    if shift:
      scale = scale + 0.2
    else:
      # translate points up
      path_y = [y - 1 for y in path_y]
      
  def down_cb(shift):
    global scale, path_y
    if shift:
      scale = scale - 0.2
    else:
      # translate points down
      path_y = [y + 1 for y in path_y]    
      
  def left_cb(shift):
    global rot, path_x
    if shift:
      rot = rot + 0.2
    else:
      # translate points to the left
      path_x = [x - 1 for x in path_x]
      
  def right_cb(shift):
    global rot, path_y
    if shift:
      rot = rot - 0.2
    else:
      # translate points to the right
      path_x = [x + 1 for x in path_x]        
  
  def on_press(shift):
    if shift:
      global shift_on
      shift_on = True
  
  def on_release(key):
    print('{0} released'.format(key))
    global shift_on
    if key == kb.Key.shift:
      shift_on = False
    elif key == kb.Key.left:
      left_cb(shift_on)
    elif key == kb.Key.right:
      right_cb(shift_on)
    elif key == kb.Key.down:
      down_cb(shift_on)
    elif key == kb.Key.up:
      up_cb(shift_on)
    elif key == kb.Key.enter:
      global enter_pressed
      enter_pressed = True
      
    return False

  #listener = kb.Listener(on_press=on_press, on_release=on_release)
  #listener.start()
  
  print("use the arrow keys and shift to rotate, translate, & scale the path")
  
  # load the picture of the map
  map_img = None
  with open("/tmp/map.pgm", 'rb') as pgmf:
    map_img = plt.imread(pgmf)
    
  loop_count = 0  
  while not enter_pressed:
    loop_count = loop_count + 1
    plt.clf()
    map_plot = plt.imshow(map_img)
    trans_path_x = [x * scale * math.sin(rot) for x in path_x]
    trans_path_y = [y * scale * math.cos(rot) for y in path_y]
    # overlay the path on the map 
    plt.scatter(x=trans_path_x, y=trans_path_y, c='r', s=3)
    #if loop_count > 1:
    plt.draw()
    #plt.show(block=False)
    with kb.Listener(on_press=on_press, on_release=on_release) as listener:
      listener.join() 
    #time.sleep(1.0)
  
  # add buffer to map to enable rotate + crop
  # crop with robot position at center
  # rotate, then crop to final size
  # draw robot's pose (start simple, like an arrow)
  
  # restart and loop over the whole bag
  bag = rosbag.Bag('/tmp/loc.bag')
  path_x, path_y = [], []
  for topic, msg, t in bag.read_messages(topics=['/tf']):
    # print(msg)
    x,y = 0,0
    path_x.append(x)
    path_y.append(y)
  bag.close()
  
  # save in the format Tim's already using (i.e. csv)
  # save cropped image of map & resized camera image
  
  # === FUTURE FEATURES ===
  # Include history channels (i.e. multiple stacked FPV images)
  # Generating GMP images using the blueprint. 
  # ^ Requires scaling offline or as additional script setting.
