import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg
from control_msgs.msg import JointTrajectoryControllerState
from tmc_control_msgs.msg import (
    GripperApplyEffortAction,
    GripperApplyEffortGoal)
import actionlib
from actionlib_msgs.msg import GoalStatus

class ArmControl(object):
    """ Move arm on button clicks """

    def __init__(self):

        grasp_action = '/hsrb/gripper_controller/grasp'

        self._gripper_control_client = actionlib.SimpleActionClient(
            grasp_action, GripperApplyEffortAction)

        # Subscribe arm state data from HSR
        self.arm_state_sub = rospy.Subscriber('/hsrb/arm_trajectory_controller/state', JointTrajectoryControllerState,
                                           self.receive_state)
        # Wait until connection
        rospy.wait_for_message('/hsrb/arm_trajectory_controller/state', JointTrajectoryControllerState, timeout=100.0)
        # Store positions received from topic
        self.actual_positions = [0, 0, 0, 0, 0]
        self.desired_positions = [0, 0, 0, 0, 0]

        # Create publisher
        self._pub = rospy.Publisher('/hsrb/arm_trajectory_controller/command', trajectory_msgs.msg.JointTrajectory,
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
                if c.name == 'arm_trajectory_controller' and c.state == 'running':
                    running = True


    def clicked_arm_raise(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.67, -0.4, 0.0, 0.0, 0.0]
        p.time_from_start = rospy.Time(5)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)


    def clicked_arm_lower(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.0, -0.0, 0.0, -1.5, 0.0]
        p.time_from_start = rospy.Time(5)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_arm_neutral(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.0, -0.0, -1.5, -1.5, 0.0]
        p.time_from_start = rospy.Time(5)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_at_target(self, target_z):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [target_z - 0.6, -0.3, 0.0, -1.5, 0.0]
        p.time_from_start = rospy.Time(5)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_raise_trunk(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0]+.1, -0.25, self.actual_positions[2], self.actual_positions[3],
                       self.actual_positions[4]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_lower_trunk(self):
        # fill ROS message
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0]-.1, -0.25, self.actual_positions[2], self.actual_positions[3],
                       self.actual_positions[4]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_right_armRoll(self):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2]-.1, self.actual_positions[3],
                       self.actual_positions[4]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_left_armRoll(self):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2]+.1, self.actual_positions[3],
                       self.actual_positions[4]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_right_wristRoll(self):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2], self.actual_positions[3],
                       self.actual_positions[4]+.1]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_left_wristRoll(self):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2], self.actual_positions[3],
                       self.actual_positions[4]-.1]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_up_wristFlex(self):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2], self.actual_positions[3]+.1,
                       self.actual_positions[4]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_down_wristFlex(self):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1], self.actual_positions[2], self.actual_positions[3]-.1,
                       self.actual_positions[4]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_up_armFlex(self):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1]+.1, self.actual_positions[2], self.actual_positions[3],
                       self.actual_positions[4]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    def clicked_down_armFlex(self):
        traj = trajectory_msgs.msg.JointTrajectory()
        traj.joint_names = ["arm_lift_joint", "arm_flex_joint", "arm_roll_joint", "wrist_flex_joint",
                            "wrist_roll_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [self.actual_positions[0], self.actual_positions[1]-.1, self.actual_positions[2], self.actual_positions[3],
                       self.actual_positions[4]]
        p.time_from_start = rospy.Time(.2)
        traj.points = [p]
        # publish ROS message
        self._pub.publish(traj)

    #hand_movement for grasping and release
    def hand_movement(self, effort):
        goal = GripperApplyEffortGoal()
        goal.effort = effort

        if (self._gripper_control_client.send_goal_and_wait(goal) ==
                GoalStatus.SUCCEEDED):
            return True
        else:
            return False

    def clicked_hand_close(self):
        self.hand_movement(-.01)
        rospy.sleep(.2)


    def clicked_hand_open(self):
        self.hand_movement(+.01)
        rospy.sleep(.2)

    def receive_state(self, data):
        self.actual_positions = data.actual.positions
        self.desired_positions = data.desired.positions

    def shutdown(self):
        self._pub.publish(trajectory_msgs.msg.JointTrajectory())
