from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPen, QPixmap
from PyQt5.QtCore import Qt, QPoint, QTimer
import rospy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
import tf.transformations
import math
import arm_control, gripper_control


class AutoNavigate(QWidget):

    def __init__(self):
        super(AutoNavigate, self).__init__()
        self.map_resolution = .05  # meters per pixel
        self.map_dimensions = (200, 200)#(600, 600) #(2048, 2048) (1800, 1800) #(200, 200)  # columns, rows

        self.center = self.map_dimensions[0] / 2, self.map_dimensions[1] / 2

        self.canvas_pose = (10, 10)  # variable to store robot's coordinates on the pixmap
        self.global_pose = (0, 0)  # variable to store robot's coordinates on the pixmap
        self.goal_canvas_pose = (0, 0)  # variable to store goal coordinates on the pixmap
        self.traveling = False
        self.scale = 2.5 #2.5#0.8  # how big you want the map to show up
        self.mousePressEvent = self.get_pos
        self.armMove = arm_control.ArmControl()
        self.gripper_move = gripper_control.GripperControl()

        self.global_pose_sub = rospy.Subscriber('/global_pose', PoseStamped, self.receive_state)
        rospy.wait_for_message('/global_pose', PoseStamped, timeout=5.0)

        self.goal_pub = rospy.Publisher('goal', PoseStamped, queue_size=10)
        # wait to establish connection between the navigation interface
        while self.goal_pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        self.initUI()
        self.timer = QTimer()
        self.timer.timeout.connect(self.repaint)
        self.timer.start(500)

    def initUI(self):
        self.setGeometry(0, 0, self.map_dimensions[0] * self.scale, self.map_dimensions[1] * self.scale)
        # We can add a Qtcore.timer here to call update() in every x ms
        self.show()

    def paintEvent(self, event):
        #pixmap = QPixmap("class-map-v2/map_cropped_tagged3.pgm")  # cropped pgm must have the same center as original
        #pixmap = QPixmap("nutrition_map_v2/nutrition_map_labeled.jpg")
        pixmap = QPixmap("LAB_MAP2022/lab-map.jpg")
        #pixmap = QPixmap("map_May11/map_cropped_labeled.pgm")
        #pixmap = QPixmap("map_July29/map.pgm")
	#pixmap = QPixmap("labeled_SBSG_Hallways/Cropped_Labled_map.pgm")
        size = pixmap.size()
        scaledPix = pixmap.scaled(size * self.scale, Qt.KeepAspectRatio)

        painter = QPainter(self)
        painter.drawPixmap(0, 0, scaledPix, 0, 0, self.map_dimensions[0] * self.scale,
                           self.map_dimensions[1] * self.scale)
        pen = QPen(Qt.red, 3)
        painter.setPen(pen)
        painter.drawEllipse(QPoint(self.canvas_pose[0], self.canvas_pose[1]), 4 * self.scale, 4 * self.scale)
        if self.traveling:
            pen2 = QPen(Qt.green, 3)
            painter.setPen(pen2)
            painter.drawEllipse(QPoint(self.goal_canvas_pose[0], self.goal_canvas_pose[1]), 4 * self.scale,
                                4 * self.scale)

    def receive_state(self, data):
        self.global_pose = (data.pose.position.x, data.pose.position.y)
        self.canvas_pose = self.convert_global_to_canvas(self.global_pose)
        if self.traveling:
            if math.sqrt(math.pow((self.canvas_pose[0]-self.goal_canvas_pose[0]), 2)+math.pow((self.canvas_pose[1]-self.goal_canvas_pose[1]), 2))/self.scale < 2:
                self.traveling = False
        #self.repaint()
        #self.update()

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

    def get_pos(self, event):
        self.goal_canvas_pose = (event.pos().x(), event.pos().y())
        goal_pose = self.convert_canvas_to_global((event.pos().x(), event.pos().y()))
        print('Going to: ' + str(goal_pose))
        self.traveling = True
        self.repaint()

        # fill ROS message
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position = Point(goal_pose[0], goal_pose[1], 0)
        # goal yaw
        quat = tf.transformations.quaternion_from_euler(0, 0, math.atan2(goal_pose[1] - self.global_pose[1], goal_pose[0] - self.global_pose[0]))
        goal.pose.orientation = Quaternion(*quat)

        # publish ROS message
        self.goal_pub.publish(goal)
        self.armMove.clicked_arm_neutral()

    def go_to_mapXY(self, target_x, x_offset, target_y, y_offset):
        x = target_x + x_offset
        y = target_y + y_offset
        print('Going to: ' + str(x) + ' ' + str(y))
        self.traveling = True
        self.repaint()

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

        #while math.sqrt(math.pow((x - self.global_pose[0]), 2)+math.pow((y - self.global_pose[1]), 2)) > 0.1:
        #    rospy.sleep(0.2)
