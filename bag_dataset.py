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
_help = "reuse previous path calculations"
parser.add_argument('--reuse_path', default=False, action='store_true', help=_help)
args = parser.parse_args()
assert args.map_file != "" and args.bag_file != "", "Must specify path to *.pbstream & *.bag files!"

if __name__ == "__main__":
  if not args.reuse_path:
    # run a bag in offline localization-only mode (requires a previously learned SLAM map)
    os.system("rosparam set use_sim_time true")

    os.system("pkill cart")
    # carl_localize.launch expects the map to be here
    os.system("cp " + args.map_file + " /tmp/current.pbstream")
    os.system("roslaunch cartographer_toyota_hsr carl_localize.launch &")
    # could ask user to provide this, but we have the .pbstream anyway
    os.system("rosrun map_server map_saver --occ 49 --free 40 -f '/tmp/map'")
    os.system("pkill cart")

    # now run cart in offline mode
    offline_cmd = "roslaunch cartographer_toyota_hsr carl_offline.launch bag_filenames:='" 
    offline_cmd = offline_cmd + args.bag_file + "' save_file:='/tmp/offline.pbstream'"
    os.system(offline_cmd)
    os.system("cp /tmp/offline.pbstream /tmp/current.pbstream")

    os.system("roslaunch cartographer_toyota_hsr carl_localize.launch &")
    time.sleep(3)
    os.system("rosservice call /trajectory_query 'trajectory_id: 1' > /tmp/traj.txt")
    time.sleep(3)
    os.system("pkill cart")
    # get just the pose position (x,y) and the corresponding timestamp (secs)
    os.system("grep -C4 position /tmp/traj.txt | grep -e 'x:' > /tmp/x.log")
    os.system("grep -C4 position /tmp/traj.txt | grep -e 'y:' > /tmp/y.log")
    os.system("grep -C4 position /tmp/traj.txt | grep -e 'secs:' | grep -v 'nsecs' > /tmp/secs.log")
    os.system("grep -C4 position /tmp/traj.txt | grep -e 'nsecs:' > /tmp/nsecs.log")

  # read the 4 files into parallel lists
  path_x, path_y = [], []
  path_secs, path_nsecs = [], []
  with open('/tmp/x.log', 'r') as x_file:
    lines = x_file.readlines()

  for l in lines:
    l = l.strip()
    path_x.append(float(l[3:]))

  with open('/tmp/y.log', 'r') as y_file:
    lines = y_file.readlines()

  for l in lines:
    l = l.strip()
    path_y.append(-1 * float(l[3:]))

  with open('/tmp/secs.log', 'r') as secs_file:
    lines = secs_file.readlines()

  for l in lines:
    l = l.strip()
    path_secs.append(float(l[6:]))

  with open('/tmp/nsecs.log', 'r') as nsecs_file:
    lines = nsecs_file.readlines()

  for l in lines:
    l = l.strip()
    path_nsecs.append(float(l[6:]))

  for i in range(len(path_secs)):
    path_secs[i] = path_secs[i] + path_nsecs[i] / 1e9
  path_nsecs = [] # don't need nsecs anymore
  
  # use keys to translate, rotate, & scale the path
  print("specific settings for SBSG 2nd floor")
  for i in range(len(path_x)):
    path_x[i] = path_x[i] + 53
    path_y[i] = path_y[i] + 5
  rot = 0.04 # radians
  scale = 126.5
  shift_on = False
  enter_pressed = False
  
  def up_cb(shift):
    global scale, path_y
    if shift:
      scale = scale + 0.15
    else:
      # translate points up
      path_y = [y - .1 for y in path_y]
      
  def down_cb(shift):
    global scale, path_y
    if shift:
      scale = scale - 0.15
    else:
      # translate points down
      path_y = [y + .1 for y in path_y]    
      
  def left_cb(shift):
    global rot, path_x
    if shift:
      rot = rot + 0.003
    else:
      # translate points to the left
      path_x = [x - .1 for x in path_x]
      
  def right_cb(shift):
    global rot, path_x
    if shift:
      rot = rot - 0.003
    else:
      # translate points to the right
      path_x = [x + .1 for x in path_x]        
  
  def on_press(key):
    if key == kb.Key.shift:
      global shift_on
      shift_on = True
  
  def on_release(key):
    # print('{0} released'.format(key))
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
  
  print("use the arrow keys and shift to rotate, translate, & scale the path")
   
  # load the picture of the map
  map_img = None
  # with open("/tmp/map.pgm", 'rb') as pgmf:
  # @TODO hardcoded for SBSG 2nd floor
  with open("/tmp/test.png", 'rb') as pgmf:
    map_img = plt.imread(pgmf)
    
  print("path_x[0] start: ", path_x[0])  
  print("path_y[0] start: ", path_y[0])  
  while not enter_pressed:
    plt.clf()
    plt.imshow(map_img, resample=False)
    trans_path_x, trans_path_y = [], []
    for i in range(len(path_x)):
      x = path_x[i] * math.cos(rot) - path_y[i] * math.sin(rot)
      trans_path_x.append(x * scale)
      y = path_y[i] * math.cos(rot) + path_x[i] * math.sin(rot)
      trans_path_y.append(y * scale)
    # overlay the path on the map 
    plt.scatter(x=trans_path_x, y=trans_path_y, c='b', s=3)
    plt.show(block=False)
    plt.pause(0.01)
    with kb.Listener(on_press=on_press, on_release=on_release) as listener:
      listener.join() 
  
  print("path_x[0] end: ", path_x[0])
  print("path_y[0] end: ", path_y[0])
  print("scale: ", scale)
  print("rot: ", rot)
  
  # add buffer to map to enable rotate + crop
  # crop with robot position at center
  # rotate, then crop to final size
  # draw robot's pose (start simple, like an arrow)
  
  # save in the format Tim's already using (i.e. csv)
  # save cropped image of map & resized camera image
  # loop over the bag to save each image with it's pose
  print("EXITING")
  exit()
  bag = rosbag.Bag('/tmp/loc.bag')
  path_x, path_y = [], []
  for topic, msg, t in bag.read_messages(topics=['/image_proc_resize/image']):
    print(msg)
  bag.close()
  
  # === FUTURE FEATURES ===
  # Include history channels (i.e. multiple stacked FPV images)
  # Generating GMP images using the blueprint images instead of map.pgm 
