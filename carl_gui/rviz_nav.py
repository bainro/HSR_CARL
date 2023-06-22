#!/usr/bin/env python3

import tf
import rospy
import roslib
from nav_msgs.msg import OccupancyGrid

map_pub = None

def callback(data):
    data.info.origin.position.x = data.info.origin.position.x + 5
    data.info.origin.position.y = data.info.origin.position.y + 5 
    map_pub.publish(data)

def listener():
    rospy.Subscriber('/map', OccupancyGrid, callback)
    # keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rviz_cart_offline', anonymous=True)
    map_pub = rospy.Publisher('/rviz_map', OccupancyGrid, queue_size=10)
    listener()
