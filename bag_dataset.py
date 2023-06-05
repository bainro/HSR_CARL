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
from tf import TransformerROS as tfROS
from geometry_msgs.msg import PoseStamped

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
  
  # carl_localize.launch expects the map to be here
  os.system("cp " + args.map_file + " /tmp/current.pbstream")
  os.system("roslaunch cartographer_toyota_hsr carl_offline.launch &")
  # could ask user to provide this, but we have the .pbstream anyway
  os.system("rosrun map_server map_saver --occ 49 --free 40 -f '/tmp/map'")
  os.system("pkill cart")
  
  # now run cart in offline mode
  offline_cmd = "roslaunch cartographer_toyota_hsr carl_offline.launch bag_filenames:='" 
  offline_cmd = offline_cmd + args.bag_file + "' save_file:='/tmp/test.pbstream' &"
  os.system(offline_cmd)

  os.system("roslaunch cartographer_toyota_hsr carl_offline.launch &")
  # os.system("rosservice call /trajectory_query 'trajectory_id: 1' &>/tmp/robot_traj.txt")
  # os.system("grep -C4 position /tmp/traj.txt | grep -e 'secs' -e 'x:' -e 'y:' | grep -v 'nsecs'")
  
  def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
      try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        return resp1.sum
      except rospy.ServiceException as e:
          print("Service call failed: %s"%e)
  
  path_x, path_y = [], []
  
  # use keys to translate, rotate, & scale the path
  rot = 0 # rotation factor in radians
  scale = 1
  shift_on = False
  enter_pressed = False
  
  def up_cb(shift):
    global scale, path_y
    if shift:
      scale = scale + 2
    else:
      # translate points up
      path_y = [y - 10 for y in path_y]
      
  def down_cb(shift):
    global scale, path_y
    if shift:
      scale = scale - 2
    else:
      # translate points down
      path_y = [y + 10 for y in path_y]    
      
  def left_cb(shift):
    global rot, path_x
    if shift:
      rot = rot + 0.2
    else:
      # translate points to the left
      path_x = [x - 10 for x in path_x]
      
  def right_cb(shift):
    global rot, path_x
    if shift:
      rot = rot - 0.2
    else:
      # translate points to the right
      path_x = [x + 10 for x in path_x]        
  
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
  with open("/tmp/map.pgm", 'rb') as pgmf:
    map_img = plt.imread(pgmf)
    
  while not enter_pressed:
    plt.clf()
    plt.imshow(map_img)
    trans_path_x = [x * scale * math.sin(rot) for x in path_x]
    trans_path_y = [y * scale * math.cos(rot) for y in path_y]
    # overlay the path on the map 
    plt.scatter(x=trans_path_x, y=trans_path_y, c='r', s=3)
    plt.show(block=False)
    plt.pause(0.01)
    with kb.Listener(on_press=on_press, on_release=on_release) as listener:
      listener.join() 
  
  # add buffer to map to enable rotate + crop
  # crop with robot position at center
  # rotate, then crop to final size
  # draw robot's pose (start simple, like an arrow)
  
  # save in the format Tim's already using (i.e. csv)
  # save cropped image of map & resized camera image
  # loop over the bag to save each image with it's pose
  bag = rosbag.Bag('/tmp/loc.bag')
  path_x, path_y = [], []
  for topic, msg, t in bag.read_messages(topics=['/image_proc_resize/image']):
    print(msg)
  bag.close()
  
  # === FUTURE FEATURES ===
  # Include history channels (i.e. multiple stacked FPV images)
  # Generating GMP images using the blueprint images instead of map.pgm 
