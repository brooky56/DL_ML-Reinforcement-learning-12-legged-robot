#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *


# Initializes everything
def start_conf(joints_arr_l, joints_arr_r):
    """
    Configure initial position of the system

    :param: joints_arr_l, joints_arr_r: joints array that include joints that should move, data - gamepad buttons and sticks axis values
    :return None
    """

    start_str_left = JointState()
    start_str_left.header = Header()
    start_str_left.header.stamp = rospy.Time.now()
    start_str_left.name = joints_arr_l

    start_pos = 0.0
    start_str_left.position = [start_pos, start_pos, start_pos, start_pos, start_pos, start_pos]
    start_str_left.velocity = []
    start_str_left.effort = []

    start_str_right = JointState()
    start_str_right.header = Header()
    start_str_right.header.stamp = rospy.Time.now()
    start_str_right.name = joints_arr_l

    start_pos = 0.0
    start_str_right.position = [start_pos, start_pos, start_pos, start_pos, start_pos, start_pos]
    start_str_right.velocity = []
    start_str_right.effort = []
    

    rospy.Publisher('/strirus_robot/joint_states', JointState, queue_size=10).publish(start_str_left)
    rospy.Publisher('/strirus_robot/joint_states', JointState, queue_size=10).publish(start_str_right)   

if __name__ == '__main__':
    # Left joints array
    joints_arr_l = []
    # Right joints array
    joints_arr_r = []

    for i in range(6):
        str_l = "/strirus_robot/joint{0}_position_controller_l/command".format(i)
        str_r = "/strirus_robot/joint{0}_position_controller_r/command".format(i)
        joints_arr_l.append(str_l)
        joints_arr_r.append(str_r)

    start_conf(joints_arr_l, joints_arr_r)


    while True:
        print("Enter position for left arr:")
        pos  = float(input())

        print("Enter velocity for left arr:")
        vel = float(input())
        
        print("Enter effort for left arr:")
        effort = float(input())

        start_str_left = JointState()
        start_str_left.header = Header()
        start_str_left.header.stamp = rospy.Time.now()
        start_str_left.name = joints_arr_l

        start_str_left.position = [pos, pos, pos, pos, pos, pos]
        start_str_left.velocity = [vel, vel, vel, vel, vel, vel]
        start_str_left.effort = [effort, effort, effort, effort, effort]

        rospy.Publisher('/strirus_robot/joint_states', JointState, queue_size=10).publish(start_str_left)