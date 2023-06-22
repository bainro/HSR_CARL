# Author: H J Kashyap, T Hwu
import rospy
import controller_manager_msgs.srv
import geometry_msgs.msg
import trajectory_msgs.msg
from control_msgs.msg import JointTrajectoryControllerState
import yaml
import time
import math
import re
import numpy as np


class BaseTrajectoryControl(object):
    """ Move base to a position """

    def __init__(self):
        # Subscribe color image data from HSR
        self._base_state_sub = rospy.Subscriber('/hsrb/omni_base_controller/state', JointTrajectoryControllerState, self.receive_state)
        # Wait until connection
        rospy.wait_for_message('/hsrb/omni_base_controller/state', JointTrajectoryControllerState, timeout=100.0)
        # Store positions received from topic
        self.actual_positions = (0, 0, 0)
        self.desired_positions = (0, 0, 0)
        self.actual_vel = (0, 0, 0)
        self.desired_vel = (0, 0, 0)

        with open("param.yaml", "r") as f:
            self.param = yaml.load(f, Loader=yaml.FullLoader)

        # initialize ROS publisher
        self._pub = rospy.Publisher('/hsrb/omni_base_controller/command', trajectory_msgs.msg.JointTrajectory,
                                    queue_size=10)

        # wait to establish connection between the controller
        while self._pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                                              controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'omni_base_controller' and c.state == 'running':
                    running = True

    def clicked_left_90(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2] + 1.5708]
        p.velocities = [0.0, 0.0, 0.3]
        p.time_from_start = rospy.Time(self.param["rotate_90"]["time_to_rotate"])
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_right_90(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2] - 1.5708]
        p.velocities = [0.0, 0.0, 0.3]
        p.time_from_start = rospy.Time(self.param["rotate_90"]["time_to_rotate"])
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_left_45(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2] + 0.775]
        p.velocities = [0.0, 0.0, 0.3]
        p.time_from_start = rospy.Time(self.param["rotate_45"]["time_to_rotate"])
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_right_45(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["odom_x", "odom_y", "odom_t"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2] - 0.775]
        p.velocities = [0.0, 0.0, 0.3]
        p.time_from_start = rospy.Time(self.param["rotate_45"]["time_to_rotate"])
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_reverse(self):
        print('Trying to go back 0.5m')
        offset = -0.5
        safety = offset + (-0.5)
        target_x = self.actual_positions[0] + (offset * math.cos(self.actual_positions[2]))
        target_y = self.actual_positions[1] + (offset * math.sin(self.actual_positions[2]))

        safety_x = self.actual_positions[0] + (safety * math.cos(self.actual_positions[2]))
        safety_y = self.actual_positions[1] + (safety * math.sin(self.actual_positions[2]))

        # print('x: ', self.actual_positions[0], target_x)
        # print('y: ', self.actual_positions[1], target_y)

        if self.is_trajectory_occupied(self.actual_positions[0], self.actual_positions[1], safety_x, safety_y):
            print('cannot go back, occupied!')
            return False
        else:
            # fill ROS message
            traj = trajectory_msgs.msg.JointTrajectory()
            traj.joint_names = ["odom_x", "odom_y", "odom_t"]
            p = trajectory_msgs.msg.JointTrajectoryPoint()
            p.positions = [target_x, target_y, self.actual_positions[2]]
            p.velocities = [0.0, 0.0, 0.0]
            p.time_from_start = rospy.Time(self.param["reverse"]["0.5m"])
            traj.points = [p]
            # publish ROS message
            self._pub.publish(traj)
            return True

    def receive_state(self, data):
        self.actual_positions = data.actual.positions
        self.desired_positions = data.desired.positions
        self.actual_vel = data.actual.velocities
        self.desired_vel = data.desired.velocities

    def is_trajectory_occupied(self, x, y, target_x, target_y):
        # from matplotlib import pyplot
        # pyplot.imshow(self.pgm, pyplot.cm.gray)
        # pyplot.show()
        '''
        print "a1"
        showmap = rospy.ServiceProxy('get_static_obstacle_map', GetMap)
        print "a2"
        print showmap
        print showmap()
        print showmap().map
        occupancy = showmap().map

        data = occupancy.data
        info = occupancy.info
        '''
        map_resolution = self.param["map"]["resolution"]

        for i in range(1, 20):
            # print((x + (target_x - x)*i/20), (y + (target_y - y)*i/20))
            x_map = (x + (target_x - x)*i/20) / map_resolution
            y_map = (y + (target_y - y)*i/20) / map_resolution
            r_map = int(2048 / 2.0 - y_map)
            c_map = int(2048 / 2.0 + x_map)
            # print(r_map, c_map)

            if self.pgm[r_map, c_map] == 205 or self.pgm[r_map, c_map] == 0:
                print ("True")
                return True
        print ("False")
        return False

    def read_pgm(self, filename, byteorder='>'):
        """Return image data from a raw PGM file as numpy array.

        Format specification: http://netpbm.sourceforge.net/doc/pgm.html

        """
        with open(filename, 'rb') as f:
            buffer = f.read()
        try:
            header, width, height, maxval = re.search(
                b"(^P5\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
        except AttributeError:
            raise ValueError("Not a raw PGM file: '%s'" % filename)
        return np.frombuffer(buffer,
                             dtype='u1' if int(maxval) < 256 else byteorder + 'u2',
                             count=int(width) * int(height),
                             offset=len(header)
                             ).reshape((int(height), int(width)))

    def shutdown(self):
        self._pub.publish(trajectory_msgs.msg.JointTrajectory())


class BaseControl(object):
    """ Move base with a velocity on button clicks """

    def __init__(self):
        # Create publisher
        self._pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)
        # Wait to establish connection between the controller
        while self._pub.get_num_connections() == 0:
            rospy.sleep(0.1)
        # Make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                                              controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'omni_base_controller' and c.state == 'running':
                    running = True

    def clicked_forward(self):
        # fill ROS message
        tw = geometry_msgs.msg.Twist()
        tw.linear.x =0.2
        # publish ROS message
        self._pub.publish(tw)

    # def clicked_reverse(self):
    #     # fill ROS message
    #     tw = geometry_msgs.msg.Twist()
    #     tw.linear.x = -0.02 # slow to discourage use
    #     # publish ROS message
    #     self._pub.publish(tw)

    def clicked_left(self):
        # fill ROS message
        tw = geometry_msgs.msg.Twist()
        tw.angular.z = 0.2
        # publish ROS message
        self._pub.publish(tw)

    def clicked_right(self):
        # fill ROS message
        tw = geometry_msgs.msg.Twist()
        tw.angular.z = -0.2
        # publish ROS message
        self._pub.publish(tw)

    def released(self):
        # fill ROS message
        tw = geometry_msgs.msg.Twist()
        # publish ROS message
        self._pub.publish(tw)

    def shutdown(self):
        self._pub.publish(geometry_msgs.msg.Twist())

class HeadControl(object):
    """ Move head on button clicks """

    def __init__(self):
        # Subscribe color image data from HSR
        self._image_sub = rospy.Subscriber('/hsrb/head_trajectory_controller/state', JointTrajectoryControllerState, self.receive_state)
        # Wait until connection
        rospy.wait_for_message('/hsrb/head_trajectory_controller/state', JointTrajectoryControllerState, timeout=10.0)
        # Store positions received from topic
        self.actual_positions = (0, 0)
        self.desired_positions = (0, 0)

        # Create publisher
        self._pub = rospy.Publisher('/hsrb/head_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory,
                                    queue_size=10)
        # Wait to establish connection between the controller
        while self._pub.get_num_connections() == 0:
            rospy.sleep(0.1)

        # Make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
                                              controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'head_trajectory_controller' and c.state == 'running':
                    running = True

    def clicked_up(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[1], self.actual_positions[0]+.1]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_down(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[1], self.actual_positions[0]-.1]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_left(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[1]+.1, self.actual_positions[0]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_right(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[1]-.1, self.actual_positions[0]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_home(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0,0]
        p.time_from_start = rospy.Time(1)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_left_45(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[1] + 0.775, self.actual_positions[0]]
        p.time_from_start = rospy.Time(2.5)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_right_45(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[1] - 0.775, self.actual_positions[0]]
        p.time_from_start = rospy.Time(2.5)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def receive_state(self, data):
        self.actual_positions = data.actual.positions
        self.desired_positions = data.desired.positions

    def shutdown(self):
        self._pub.publish(trajectory_msgs.msg.JointTrajectory())


# class GripperControl(object):
#     """ Move gripper on button clicks """
#
#     def __init__(self):
#
#         # Create publisher
#         self._pub = rospy.Publisher('/hsrb/gripper_controller/command', trajectory_msgs.msg.JointTrajectory,
#                                     queue_size=10)
#         # Wait to establish connection between the controller
#         while self._pub.get_num_connections() == 0:
#             rospy.sleep(0.1)
#         # Make sure the controller is running
#         rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
#         list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',
#                                               controller_manager_msgs.srv.ListControllers)
#         running = False
#         while running is False:
#             rospy.sleep(0.1)
#             for c in list_controllers().controller:
#                 if c.name == 'gripper_controller' and c.state == 'running':
#                     running = True
#
#     def clicked_hand_gripper(self):
#         # fill ROS message
#         traj = trajectory_msgs.msg.JointTrajectory()
#         traj.joint_names = ["hand_motor_joint"]
#         p = trajectory_msgs.msg.JointTrajectoryPoint()
#         p.positions = [-0.5]
#         p.velocities = [0.1]
#         p.effort = [0.1]
#         p.time_from_start = rospy.Time(3)
#         traj.points = [p]
#         # publish ROS message
#         self._pub.publish(traj)
#
#     def clicked_gripper_open(self):
#         # fill ROS message
#         traj = trajectory_msgs.msg.JointTrajectory()
#         traj.joint_names = ["hand_motor_joint"]
#         p = trajectory_msgs.msg.JointTrajectoryPoint()
#         p.positions = [1.2]
#         p.velocities = [0.1]
#         p.effort = [0.1]
#         p.time_from_start = rospy.Time(3)
#         traj.points = [p]
#         # publish ROS message
#         self._pub.publish(traj)
#
#     def clicked_gripper_close(self):
#         # fill ROS message
#         traj = trajectory_msgs.msg.JointTrajectory()
#         traj.joint_names = ["hand_motor_joint"]
#         p = trajectory_msgs.msg.JointTrajectoryPoint()
#         p.positions = [-1.0]
#         p.velocities = [0.1]
#         p.effort = [0.1]
#         p.time_from_start = rospy.Time(3)
#         traj.points = [p]
#         # publish ROS message
#         self._pub.publish(traj)
