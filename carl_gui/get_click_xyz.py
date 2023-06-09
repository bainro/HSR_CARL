#!/usr/bin/env python
import tf
import rospy
import numpy as np
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

# camera intrinsic parameters
K_d = np.array([537.2190506060545, 0.0, 322.6469894848447, 0.0, 537.6207002715853, 234.9387117429966, 0.0, 0.0, 1.0])
K_d = K_d.reshape(3, 3)
cx_d = K_d[0, 2]
cy_d = K_d[1, 2]
fx_inv_d = 1.0/K_d[0, 0]
fy_inv_d = 1.0/K_d[1, 1]

class Get3Dcoordinates(object):
    """Read a point cloud snapshot and returns the XYZ coordinates of a corresponding pixel location"""

    def __init__(self):
        self.rgb_h = 0
        self.rgb_v = 0
        self.found_3d = False

        topic_name = '/hsrb/head_rgbd_sensor/depth_registered/image_raw'
        self._bridge = CvBridge()

        self.rgbd_point = PointStamped()
        self.rgbd_point.header.frame_id = "head_rgbd_sensor_rgb_frame"

        # Subscribe color image data from HSR
        self._depth_sub = rospy.Subscriber(topic_name, Image, self._depth_image_cb)
        rospy.wait_for_message(topic_name, Image, timeout=20.0)

        self.listener = tf.TransformListener()
        self.listener.waitForTransform("map", "head_rgbd_sensor_rgb_frame", rospy.Time(0), rospy.Duration(1))

    def _depth_image_cb(self, data):
        try:
            self.depthmap = self._bridge.imgmsg_to_cv2(data, "32FC1")

        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

    def twoD_to_threeD(self, rgb_h, rgb_v):
        self.found_3d = False

        u = rgb_h
        v = rgb_v

        depth_from_rgbd = self.depthmap[rgb_v, rgb_h]
        if depth_from_rgbd <= 0.0:
            print("depth was negative :(")
            return

        try:
            Z = depth_from_rgbd * 0.001  # convert mm to meters
            print('depth: ', Z)
            X = Z * ((u - cx_d) * fx_inv_d)
            Y = Z * ((v - cy_d) * fy_inv_d)
            self.rgbd_point.header.stamp = rospy.Time(0)
            self.rgbd_point.point.x = X
            self.rgbd_point.point.y = Y
            self.rgbd_point.point.z = Z
            self.map_point = self.listener.transformPoint("map", self.rgbd_point)
            self.found_3d = True
        except:
            print ('cannot transform target position')
