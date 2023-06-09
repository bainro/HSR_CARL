'''
Coverts a rosbag into a dataset for Perspective Transorming VAE(s)
'''
import os
import cv2
import time
import math
import rosbag
import argparse
import numpy as np
import matplotlib.cm as cm
import matplotlib.pyplot as plt
from pynput import keyboard as kb
from tf import transformations as t

parser = argparse.ArgumentParser()
_help = "path to previously learned cartographer map (*.pbstream)"
parser.add_argument("--map_file", type=str, required=True, help=_help)
_help = "path to previously recorded rosbag"
parser.add_argument("--bag_file", type=str, required=True, help=_help)
_help = "directory to save the training data"
parser.add_argument("--out_dir", type=str, required=True, help=_help)
_help = "reuse previous path calculations"
parser.add_argument('--reuse_path', default=False, action='store_true', help=_help)
_help = "append to previous dataset instead of overwriting"
parser.add_argument('--combine', default=False, action='store_true', help=_help)
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
  os.system("grep -C4 'w:' /tmp/traj.txt | grep -e 'z:' > /tmp/rot_z.log")
  os.system("grep     'w:' /tmp/traj.txt > /tmp/rot_w.log")

  # read the files into parallel lists
  path_x, path_y = [], []
  # quarternion rotation, but we only need 2
  path_z, path_w = [], []
  path_yaw = []
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
    
  with open('/tmp/rot_z.log', 'r') as rot_z_file:
    lines = rot_z_file.readlines()

  for l in lines:
    l = l.strip()
    path_z.append(float(l[3:]))
    
  with open('/tmp/rot_w.log', 'r') as rot_w_file:
    lines = rot_w_file.readlines()

  for l in lines:
    l = l.strip()
    path_w.append(float(l[3:]))
    
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
  del path_nsecs # don't need nsecs anymore
  
  # change angle in radians to filter consecutive poses
  filter_dr = 0.23 # 0.3 rads ~= 17 degs
  # relative path width change to filter consecutive poses
  rel_filter_dx = 0.003 # hyperparameter to tune for each map
  path_width = max(path_x) - min(path_x)
  filter_dx = path_width * rel_filter_dx
  # last non-filtered (i.e. included) pose. 
  last_pt = [math.inf, math.inf, math.inf] 
  del_count = 0
  # filter out poses based on (dx, dr) wrt last included pose
  for i in range(len(path_secs)):
    i = i - del_count
    x = path_x[i]
    y = path_y[i]
    # triangle maths
    dx = (((last_pt[0] - x) ** 2) + ((last_pt[1] - y) ** 2)) ** 0.5
  
    qz = path_z[i]
    qw = path_w[i]
    _r, _p, yaw = t.euler_from_quaternion([0, 0, qz, qw])
    yaw = yaw + math.pi # make smallest possible value == 0
    # have to check for wrap around!
    if abs(yaw - last_pt[2]) > math.pi:
      dr = 2 * math.pi - abs(yaw - last_pt[2]) 
    else:
      dr = abs(yaw - last_pt[2])
  
    if dx > filter_dx or dr > filter_dr:
      last_pt = [x, y, yaw]
      path_yaw.append(yaw)
    else: # bye-bye!
      del path_x[i], path_y[i], path_z[i], path_w[i], path_secs[i]
      del_count = del_count + 1
  
  print("Number of datapoints after filtering: ", len(path_x))
  assert len(path_secs) == len(path_x) == len(path_y), "No longer parallel lists!"
  assert len(path_y) == len(path_z) == len(path_w), "No longer parallel lists!"
  
  # use keys to translate, rotate, & scale the path
  print("specific settings for SBSG 1st floor")
  for i in range(len(path_x)):
    path_x[i] = path_x[i] + 24.48
    path_y[i] = path_y[i] + 9.5
  rot = 0.006989 # radians
  scale = 19.9
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
  # @TODO hardcoded for SBSG 2nd floor
  # with open("/tmp/test.png", 'rb') as pgmf:
  with open("/tmp/map.pgm", 'rb') as pgmf:
    map_img = plt.imread(pgmf)
    
  print("path_x[0] start: ", path_x[0])  
  print("path_y[0] start: ", path_y[0])  
  colors = cm.gist_rainbow(np.linspace(0, 0.85, len(path_x)))
  fig = plt.figure(figsize=(36,12))
  while not enter_pressed:
    plt.clf()
    plt.imshow(map_img, resample=False, interpolation='none', cmap='gray', vmin=0, vmax=255)
    trans_path_x, trans_path_y = [], []
    for i in range(len(path_x)):
      x = path_x[i] * math.cos(rot) - path_y[i] * math.sin(rot)
      trans_path_x.append(x * scale)
      y = path_y[i] * math.cos(rot) + path_x[i] * math.sin(rot)
      trans_path_y.append(y * scale)
    # overlay the path on the map 
    plt.scatter(x=trans_path_x, y=trans_path_y, c=colors, s=3)
    plt.show(block=False)
    plt.pause(0.01)
    with kb.Listener(on_press=on_press, on_release=on_release) as listener:
      listener.join() 
  
  plt.scatter(x=trans_path_x[0], y=trans_path_y[0], c=colors[0], s=25, label="start")
  plt.scatter(x=trans_path_x[-1], y=trans_path_y[-1], c=colors[-1], s=25, label="end")
  l = plt.legend(loc="lower right", fontsize=15)
  # hack to scale legend's icons with bigger font size
  l.legendHandles[0]._sizes = [200]
  l.legendHandles[1]._sizes = [200]
  plt.show()
  fig.savefig('/tmp/overlay.svg', format='svg', dpi=1200)
  plt.clf()
  
  print("path_x[0] end: ", path_x[0])
  print("path_y[0] end: ", path_y[0])
  print("scale: ", scale)
  print("rot: ", rot)
 
  def rotate_image(image, x, y, qz, qw):
    # print("assumes HxWxC image format!")
    x = int(x//1)
    y = int(y//1)
    rotation_pt = (x,y)
    _roll, _pitch, yaw = t.euler_from_quaternion([0, 0, qz, qw])
    yaw = yaw * -1 # flip rotation
    # make the robot look up instead of to the right
    yaw = yaw + (math.pi / 2)
    yaw_degs = yaw * 180 / math.pi
    rot_mat = cv2.getRotationMatrix2D(rotation_pt, yaw_degs, 1.0)
    rot_img = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return rot_img
  
  os.makedirs(args.out_dir, exist_ok=True)
  target_size = 256
  # region of interest's (i.e. centered at robot) relative width
  roi_rel_w = 0.12 # hyperparameter to be set by user
  # print("assumes HxWxC image format!")
  rot_w = int((map_img.shape[1] * roi_rel_w) // 1)
  
  prior_data = 0
  if args.combine:
    with open(os.path.join(args.out_dir, "meta_data.csv"), "r") as meta_file:
      prior_data = len(meta_file.readlines())
  
  for c, i in enumerate(range(len(trans_path_x))):
    if len(map_img.shape) == 3: # e.g. RGB
      gmp_img = np.zeros(shape=(rot_w, rot_w, 3))
      # assumes padding color is same as top-left map px
      gmp_img[:,:,0] = map_img[0,0,0]
      gmp_img[:,:,1] = map_img[0,0,1]
      gmp_img[:,:,2] = map_img[0,0,2]
    else: # e.g. grayscale
      gmp_img = np.zeros(shape=(rot_w, rot_w)) 
      gmp_img[:,:] = map_img[0,0]
 
    rot_map = rotate_image(map_img, trans_path_x[i], trans_path_y[i], path_z[i], path_w[i])
    
    # crop out around the robot
    x_start = int(trans_path_x[i] - rot_w // 2)
    x_end = int(trans_path_x[i] + rot_w // 2)
    y_start = int(trans_path_y[i] - rot_w // 2)
    y_end = int(trans_path_y[i] + rot_w // 2)
    # x and y (blank/white) padding offset
    xpo, ypo = 0, 0
    if x_start < 0:
      xpo = abs(x_start)
      x_start = 0
    if y_start < 0:
      ypo = abs(y_start)
      y_start = 0
    # print("assumes HxWxC image format!")
    if x_end > map_img.shape[1]: 
      x_end = map_img.shape[1]
    if y_end > map_img.shape[0]:
      y_end = map_img.shape[0]
    
    # print("xpo: ", xpo)
    # print("x_start: ", x_start)
    # print("x_end: ", x_end)
    if xpo != 0: # literal edge (of map) case
      if len(rot_map.shape) == 3: # e.g. RGB
        gmp_img[:y_end-y_start, xpo+1:, :] = rot_map[y_start:y_end, x_start:x_end+1, :]
      else: # e.g. grayscale
        gmp_img[:y_end-y_start, xpo+1:] = rot_map[y_start:y_end, x_start:x_end+1]
    elif ypo != 0: # literal edge (of map) case
      if len(rot_map.shape) == 3: # e.g. RGB
        gmp_img[ypo+1:, :x_end-x_start, :] = rot_map[y_start:y_end+1, x_start:x_end, :]
      else: # e.g. grayscale
        gmp_img[ypo+1:, :x_end-x_start] = rot_map[y_start:y_end, x_start:x_end]
    else:
      if len(rot_map.shape) == 3: # e.g. RGB
        gmp_img[:y_end-y_start, :x_end-x_start, :] = rot_map[y_start:y_end, x_start:x_end, :]
      else: # e.g. grayscale
        gmp_img[:y_end-y_start, :x_end-x_start] = rot_map[y_start:y_end, x_start:x_end]
    
    gmp_img = cv2.resize(gmp_img, dsize=(target_size, target_size), 
                         interpolation=cv2.INTER_AREA) 
    cv2.imwrite(os.path.join(args.out_dir, f'{i + prior_data}_map.png'), gmp_img*255)
    if i == 0:
      plt.imshow(gmp_img, cmap='gray', vmin=0, vmax=255)
      plt.show()
  
  # save each FPV image with the corresponding GMP image
  bag = rosbag.Bag(args.bag_file)
  with open(os.path.join(args.out_dir, "meta_data.csv"), "a") as meta_data_file:
    if prior_data == 0:
      meta_data_file.write("frame,time,heading\n")
    i = 0
    for topic, msg, _t in bag.read_messages(topics=['/image_proc_resize/image']):
      if i >= len(path_x):
        break
      msg_t = msg.header.stamp.secs + (msg.header.stamp.nsecs / 1e9)
      if msg_t < path_secs[i]:
        continue
      meta_data_file.write("%s,%s,%.2f\n" % (i+prior_data, path_secs[i], path_yaw[i]))  
      assert msg.width > msg.height, "image width must be greater than image height"
      cam_img = np.asarray(list(msg.data), dtype=np.float32)
      cam_img = cam_img.reshape((msg.height, msg.width, 3))
      # crop to center
      x_offset = int((msg.width - msg.height) // 2)
      cam_img = cam_img[:, x_offset:-x_offset, :]
      assert_str = f"image should be square. {cam_img.shape[1]} != {cam_img.shape[0]}"
      assert cam_img.shape[0] == cam_img.shape[1], assert_str
      resize_dims = (target_size, target_size)
      fpv_img = cv2.resize(cam_img, dsize=resize_dims, interpolation=cv2.INTER_AREA)
      save_name = os.path.join(args.out_dir, f'{i + prior_data}_camera.png')
      cv2.imwrite(save_name, fpv_img[...,::-1])
      i = i + 1
  bag.close()
  
  print("EXITING")
  exit()
  
  # === FUTURE FEATURES ===
  # Include history channels (i.e. multiple stacked FPV images)
  # Include LIDAR as separate image channel (only affects 1st conv, which scales linearly)
