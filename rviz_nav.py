#!/usr/bin/env python3
import tf
import time
import rospy
import trajectory_msgs.msg
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tmc_base_path_planner.msg import BasePathPlanActionResult

goal_pub = None
pose_pub = None
head_pub = None

def goal_callback(data):
    # Listens for RVIZ nav goal, then remaps it to HSR's /goal topic
    goal_pub.publish(data)

def goal_listener(spin):
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    if (spin):
        rospy.spin() # keeps python running

def pose_callback(data):
    # Listens for RVIZ nav goal, then remaps it to HSR's /goal topic
    pose_pub.publish(data)

def pose_listener(spin):
    rospy.Subscriber('/tracked_pose', PoseStamped, pose_callback)
    ### @TODO
    ### Figure out how to set initalpose for cartographer from this message! start_trajectory service?
    # rospy.Subscriber('/laser_2d_correct_pose', PoseWithCovarianceStamped, init_pose_callback)
    # This will require killing more default TMC ROS nodes that publish LIDAR matches! See rqt ROS graph for details
    if (spin):
        rospy.spin()

# bring head back up to viewing height when finished navigating
def path_finish(_data):
    # fill ROS message
    traj = trajectory_msgs.msg.JointTrajectory()
    traj.joint_names = ["head_pan_joint", "head_tilt_joint"]
    p = trajectory_msgs.msg.JointTrajectoryPoint()
    p.positions = [0, 1.5]
    p.time_from_start = rospy.Time(1)
    traj.points = [p]
    # publish ROS message
    head_pub.publish(traj)

if __name__ == '__main__':
    rospy.init_node('nav_goal_listener', anonymous=True)

    # Create publisher
    head_pub = rospy.Publisher('/hsrb/head_trajectory_controller/command',
                               trajectory_msgs.msg.JointTrajectory,
                               queue_size=10)
    # Wait to establish connection between the controller
    while head_pub.get_num_connections() == 0:
        rospy.sleep(0.1)

    rospy.Subscriber("/base_path_plan/result", BasePathPlanActionResult, path_finish)

    pose_pub = rospy.Publisher('/global_pose', PoseStamped, queue_size=10)
    pose_listener(spin=False)

    goal_pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)
    goal_listener(spin=True)


    '''
    # recreating pose_integrator's /global_pose since we kill it for cartographer
    tf_listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    time.sleep(3)
    while not rospy.is_shutdown():
        (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        tmc_global_pose = PoseStamped()
        tmc_global_pose.header.stamp = rospy.Time.now()
        tmc_global_pose.header.frame_id = 'map'
        tmc_global_pose.pose.position.x = trans[0]
        tmc_global_pose.pose.position.y = trans[1]
        tmc_global_pose.pose.position.z = trans[2]
        tmc_global_pose.pose.orientation.x = rot[0]
        tmc_global_pose.pose.orientation.y = rot[1]
        tmc_global_pose.pose.orientation.z = rot[2]
        tmc_global_pose.pose.orientation.w = rot[3]
        gp_pub.publish(tmc_global_pose)
        rate.sleep()
    '''

### Not currently used
'''
from nav_msgs.msg import OccupancyGrid

def map_callback(msg):
    pub = rospy.Publisher('/map2', OccupancyGrid, queue_size=10)
    data_list = list(msg.data)
    # threshold the many values of cartographer's map to the 3 needed (-1, 0, 100)
    for i in range(len(data_list)):
        if data_list[i] < 50:
            data_list[i] = 0
        else:
            data_list[i] = 100
    msg.data = tuple(data_list)
    pub.publish(msg)

def map_listener(spin):
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    if (spin):
        rospy.spin() # keeps python running
'''
