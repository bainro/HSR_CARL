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
import matplotlib.pyplot as plt
from pynput import keyboard as kb
from tf import transformations as t

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
    os.system("grep -C4 'w:' /tmp/traj.txt | grep -e 'z:' > /tmp/rot_z.log")
    os.system("grep     'w:' /tmp/traj.txt > /tmp/rot_w.log")

  # read the files into parallel lists
  path_x, path_y = [], []
  # quarternion rotation, but we only need 2
  path_z, path_w = [], []
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
  filter_dr = 0.375 # 0.375 rads ~= 21 degs
  # relative path width change to filter consecutive poses
  rel_filter_dx = 0.1 # hyperparameter to tune for each map
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
    else: # bye-bye!
      del path_x[i], path_y[i], path_z[i], path_w[i], path_secs[i]
      del_count = del_count + 1
  
  assert len(path_secs) == len(path_x) == len(path_y), "No longer parallel lists!"
  assert len(path_y) == len(path_z) == len(path_w), "No longer parallel lists!"
  
  '''
  print("DBG ONLY!!!!!")
  skip_factor = 1
  path_x = path_x[::skip_factor]
  path_y = path_y[::skip_factor]
  path_z = path_z[::skip_factor]
  path_w = path_w[::skip_factor]
  path_secs = path_secs[::skip_factor]
  '''
  
  # use keys to translate, rotate, & scale the path
  print("specific settings for SBSG 5th floor")
  for i in range(len(path_x)):
    path_x[i] = path_x[i] + 39.40 # 53.38
    path_y[i] = path_y[i] + 28.69 # 4.8
  rot = -0.0291 # radians
  scale = 20 # 127.25
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
  # @TODO hardcoded for SBSG 5th floor
  with open("/tmp/test.pgm", 'rb') as pgmf:
    map_img = plt.imread(pgmf)
    
  print("path_x[0] start: ", path_x[0])  
  print("path_y[0] start: ", path_y[0])  
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
    plt.scatter(x=trans_path_x, y=trans_path_y, c='b', s=3)
    plt.show(block=False)
    plt.pause(0.01)
    with kb.Listener(on_press=on_press, on_release=on_release) as listener:
      listener.join() 
  
  plt.scatter(x=trans_path_x[0], y=trans_path_y[0], c='lime', s=25, label="start")
  plt.scatter(x=trans_path_x[-1], y=trans_path_y[-1], c='r', s=25, label="end")
  l = plt.legend(loc="upper right", fontsize=20)
  # hack to scale legend's icons with bigger font size
  l.legendHandles[0]._sizes = [240]
  l.legendHandles[1]._sizes = [240]
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
    yaw_degs = yaw * 180 / math.pi
    # print("FOR DBG'ING FPV ROTATION!")
    # print("yaw: ", yaw)
    # _x = 50 * math.cos(yaw)
    # _y = 50 * math.sin(yaw)
    # _x = x + int(_x//1)
    # _y = y + int(_y//1)
    # image[y:y+20,x:x+20,:] = 0
    # image[_y:_y+20,_x:_x+20,:] = 0
    # print("figure out the offset for each map's 0 degrees rotation")
    rot_mat = cv2.getRotationMatrix2D(rotation_pt, yaw_degs, 1.0)
    rot_img = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return rot_img
  
  target_size = 128
  # region of interest's (i.e. centered at robot) relative width
  roi_rel_w = 0.15 # hyperparameter to be set by user
  # print("assumes HxWxC image format!")
  rot_w = int((map_img.shape[1] * roi_rel_w) // 1)
  
  # fig2 = plt.figure(figsize=(10, 12))
  
  for c, i in enumerate(range(len(trans_path_x))):
    if len(map_img.shape) == 3: # e.g. RGB
      fpv_img = np.zeros(shape=(rot_w, rot_w, 3))
    else: # e.g. grayscale
      fpv_img = np.zeros(shape=(rot_w, rot_w))
 
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
      
    if len(rot_map.shape) == 3: # e.g. RGB
      fpv_img[ypo:y_end-y_start, xpo:x_end-x_start, :] = rot_map[y_start:y_end, x_start:x_end, :]
    else: # e.g. grayscale
      fpv_img[ypo:y_end-y_start, xpo:x_end-x_start] = rot_map[y_start:y_end, x_start:x_end]
    
    fpv_img = cv2.resize(fpv_img, dsize=(target_size, target_size), 
                         interpolation=cv2.INTER_AREA) 
    plt.imshow(fpv_img, cmap='gray', vmin=0, vmax=255)
    plt.show()
    
    '''
    if c > 15: break
    ax = plt.subplot(5,4,c+1)
    ax.axes.get_xaxis().set_visible(False)
    ax.axes.get_yaxis().set_visible(False)
    ax.imshow(fpv_img)
    '''
  
  '''
  plt.tight_layout(0.55)
  ax1 = plt.subplot(5,1,5)
  ax1.imshow(map_img)
  ax1.scatter(x=trans_path_x, y=trans_path_y, c='b', s=3)
  ax1.axes.get_xaxis().set_visible(False)
  ax1.axes.get_yaxis().set_visible(False)
  fig2.savefig('/tmp/test2.svg', format='svg', dpi=1200)
  plt.show()
  '''
  
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
  # Include LIDAR as separate image channel (only affects 1st conv, which scales linearly)
