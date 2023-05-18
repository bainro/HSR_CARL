#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from sensor_msgs.msg import Image
import controller_manager_msgs.srv
import geometry_msgs.msg
from pynput import keyboard

class MyException(Exception): pass

class RemoteView(object):
	"""Get video data from the HSR"""
	def __init__(self):
		topic_name = '/hsrb/head_rgbd_sensor/rgb/image_raw'
		self._bridge = CvBridge()
		self._input_image = None

		# Subscribe color image data from HSR
		self._image_sub = rospy.Subscriber(topic_name, Image, self._color_image_cb)
		# Wait until connection
		rospy.wait_for_message(topic_name, Image, timeout=5.0)

	def _color_image_cb(self, data):
		try:
			self._input_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
			cv2.imshow("CARL-SR Remote", self._input_image)
			cv2.waitKey(3)
		except CvBridgeError as cv_bridge_exception:
			rospy.logerr(cv_bridge_exception)

class RemoteControl(object):
	"""React to key presses"""
	def __init__(self):
		# Create publisher
		self._pub = rospy.Publisher('/hsrb/command_velocity',geometry_msgs.msg.Twist, queue_size=10)
		# Wait to establish connection between the controller		
		while self._pub.get_num_connections() == 0:
			rospy.sleep(0.1)
		# Make sure the controller is running
		rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
		list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',controller_manager_msgs.srv.ListControllers)
		running = False
		while running is False:
			rospy.sleep(0.1)
			for c in list_controllers().controller:
				if c.name == 'omni_base_controller' and c.state == 'running':
					running = True
		self._spacebar_pressed = False
		# Collect events until released
		with keyboard.Listener(on_press=self._on_press,on_release=self._on_release) as listener:
			try:
				listener.join()
			except MyException as e:
				print('{0} was pressed'.format(e.args[0]))

	def _on_press(self,key):
		if key == keyboard.Key.esc:
			raise MyException(key)
		if key == keyboard.Key.space:
			self._spacebar_pressed = True
		if key == keyboard.Key.up and self._spacebar_pressed:
			print('going forward')
			# fill ROS message
			tw = geometry_msgs.msg.Twist()
			tw.linear.x = 0.1
			# publish ROS message
			self._pub.publish(tw)
		elif key == keyboard.Key.down and self._spacebar_pressed:
			print('going backward')
			# fill ROS message
			tw = geometry_msgs.msg.Twist()
			tw.linear.x = -0.1
			# publish ROS message
			self._pub.publish(tw)
		elif key == keyboard.Key.left and self._spacebar_pressed:
			print('turning left')
			# fill ROS message
			tw = geometry_msgs.msg.Twist()
			tw.angular.z = 0.1
			# publish ROS message
			self._pub.publish(tw)
		elif key == keyboard.Key.right and self._spacebar_pressed:
			print('turning right')
			# fill ROS message
			tw = geometry_msgs.msg.Twist()
			tw.angular.z = -0.1
			# publish ROS message
			self._pub.publish(tw)
		try:
			print('alphanumeric key {0} pressed'.format(key.char))
		except AttributeError:
			print('special key {0} pressed'.format(key))

	def _on_release(self,key):
		print('{0} released'.format(key))
		if key == keyboard.Key.esc:
			# Stop listener
			return False
		if key == keyboard.Key.space:
			self._spacebar_pressed = False
		
		# stop
		tw = geometry_msgs.msg.Twist()
		self._pub.publish(tw)
def main():
	rospy.init_node('carlsr_remote')
	color_detection = RemoteView()
	keyboard_listener = RemoteControl()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down CARL-SR Remote"
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main()
