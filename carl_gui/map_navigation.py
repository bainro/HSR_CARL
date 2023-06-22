import tf
import math
import rospy
from PyQt5.QtCore import QTimer
from rviz import bindings as rviz
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from geometry_msgs.msg import Point, PoseStamped, Quaternion

class AutoNavigate(QWidget):
    def __init__(self):
        super(AutoNavigate, self).__init__()
        self.global_pose = (0, 0)  # variable to store robot's coordinates in the map frame
        self.global_pose_sub = rospy.Subscriber('/global_pose', PoseStamped, self.receive_state)
        rospy.wait_for_message('/global_pose', PoseStamped, timeout=5.0)
        self.goal_pub = rospy.Publisher('/SetGoal', PoseStamped, queue_size=10)
        while self.goal_pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, "classroom_demo.rviz")
        self.frame.load(config)
        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)
        layout = QVBoxLayout()
        layout.addWidget(self.frame)
        self.setLayout(layout)

    def receive_state(self, data):
        self.global_pose = (data.pose.position.x, data.pose.position.y)

    def go_to_mapXY(self, target_x, x_offset, target_y, y_offset):
        x = target_x + x_offset
        y = target_y + y_offset
        print('Going to: ' + str(x) + ' ' + str(y))

        # fill ROS message
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position = Point(x, y, 0)
        # goal yaw
        quat = tf.transformations.quaternion_from_euler(0, 0, math.atan2(target_y - y, target_x - x))
        goal.pose.orientation = Quaternion(*quat)

        # publish ROS message
        self.goal_pub.publish(goal)