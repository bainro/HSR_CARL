import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import rospy
import tf.transformations
from geometry_msgs.msg import Point, PoseStamped, Quaternion
import math


class AutoNavigate(QWidget):
    def __init__(self):
        self.map_resolution = .05 # meters per pixel
        self.map_dimensions = (300, 300) # columns, rows

        self.center = self.map_dimensions[0]/2, self.map_dimensions[1]/2

        self.canvas_pose = (10, 10)  # variable to store robot's coordinates on the pixmap
        self.goal_canvas_pose = (0, 0)  # variable to store goal coordinates on the pixmap
        self.traveling = False
        self.scale = 2  # how big you want the map to show up

        super(AutoNavigate, self).__init__()
        self.setGeometry(30, 30, self.map_dimensions[0]*self.scale, self.map_dimensions[1]*self.scale)
        self.mousePressEvent = self.get_pos
        self.show()

        self.global_pose_sub = rospy.Subscriber('/global_pose', PoseStamped, self.receive_state)
        rospy.wait_for_message('/global_pose', PoseStamped, timeout=5.0)

        self.goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=10)
        # wait to establish connection between the navigation interface
        while self.goal_pub.get_num_connections() == 0:
            rospy.sleep(0.1)

    def paintEvent(self, event):
        painter = QPainter(self)
        pixmap = QPixmap("map_class_cropped.pgm") # cropped pgm must have the same center as original
        painter.drawPixmap(self.rect(), pixmap)
        pen = QPen(Qt.red, 3)
        painter.setPen(pen)
        painter.drawEllipse(QPoint(self.canvas_pose[0], self.canvas_pose[1]), 2*self.scale, 2*self.scale)
        if self.traveling:
            pen2 = QPen(Qt.green, 3)
            painter.setPen(pen2)
            painter.drawEllipse(QPoint(self.goal_canvas_pose[0], self.goal_canvas_pose[1]), 2*self.scale, 2*self.scale)

    # Input: Global coord in cartesian meters
    # Output: Pixel coord in c,r
    def convert_global_to_canvas(self, global_coord):
        # convert global_coords to canvas pixels, still cartesian
        pixels = global_coord[0]/self.map_resolution, global_coord[1]/self.map_resolution
        # convert cartesian canvas pixels to c,r pixels
        crpixels = self.center[0]+pixels[0], self.center[1]-pixels[1]
        # scale up c,r pixels to fit screen and return
        return crpixels[0]*self.scale, crpixels[1]*self.scale

    def convert_canvas_to_global(self, canvas_coord):
        # unscale pixels to c,r pixels
        crpixels = canvas_coord[0]/self.scale, canvas_coord[1]/self.scale
        # convert c,r pixels to cartesian canvas pixels
        pixels = crpixels[0]-self.center[0], self.center[1]-crpixels[1]
        # convert cartesian canvas pixels to global coords and return
        return pixels[0]*self.map_resolution, pixels[1]*self.map_resolution

    def receive_state(self, data):
        global_pose = (data.pose.position.x, data.pose.position.y)
        self.canvas_pose = self.convert_global_to_canvas(global_pose)
        if self.traveling:
            if math.sqrt(math.pow((self.canvas_pose[0]-self.goal_canvas_pose[0]), 2)+math.pow((self.canvas_pose[1]-self.goal_canvas_pose[1]), 2))/self.scale < 2:
                self.traveling = False
        self.repaint()

    def get_pos(self, event):
        self.goal_canvas_pose = (event.pos().x(), event.pos().y())
        goal_pose = self.convert_canvas_to_global((event.pos().x(), event.pos().y()))
        print 'Going to: ' + str(goal_pose)
        self.traveling = True
        # fill ROS message
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position = Point(goal_pose[0], goal_pose[1], 0)
        # no goal yaw for now
        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        goal.pose.orientation = Quaternion(*quat)

        # publish ROS message
        self.goal_pub.publish(goal)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    rospy.init_node('map_mouseover')
    ex = AutoNavigate()
    sys.exit(app.exec_())
