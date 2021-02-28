#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import Float64
import numpy as np
import time

# Initializes everything
def start_conf(joints_arr_l, joints_arr_r, pub):
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
    

    pub.publish(start_str_left)
    pub.publish(start_str_right)   

def swap_phase(a, b):
    t = a
    a = b
    b = a
    return a, b

if __name__ == '__main__':

    # # Left joints array
    # joints_arr_l = []
    # # Right joints array
    # joints_arr_r = []
    pub_l = []
    pub_r = []

    for i in range(6):
        str_l = "/strirus_robot/joint{0}_position_controller_l/command".format(i)
        str_r = "/strirus_robot/joint{0}_position_controller_r/command".format(i)
        # str_l = "leg_left_{0}_revolute_joint".format(i)
        # str_r = "leg_right_{0}_revolute_joint".format(i)
        pub_l.append(rospy.Publisher(str_l, Float64, queue_size=10))
        pub_r.append(rospy.Publisher(str_r, Float64, queue_size=10))
    rospy.init_node('forward_motion')
    rate = rospy.Rate(50)

    # start_conf(joints_arr_l, joints_arr_r, pub)

    # print("Enter duration in sec:")
    # duration  = int(input())

    # abort_after = duration * 60
    # start = time.time()

    print("Enter desired position:")
    pos  = float(input())

    # print("Enter desired velocity")
    # vel = float(input())
        
    # print("Enter desired effort:")
    # effort = float(input())

    pose_phase1 = -pos 
    pose_phase2 = -pos - np.pi

    while not rospy.is_shutdown():
        # delta = time.time() - start
        
        # start_str_left = JointState()
        # start_str_left.header = Header()
        # start_str_left.header.stamp = rospy.Time.now()
        # start_str_left.name = joints_arr_l

        # Add tripod gait - 3 left 3 right (with contact) & 3 left 3 right (without contact)
        start_str_left = [pose_phase2, pose_phase1, pose_phase2, pose_phase1, pose_phase2, pose_phase1]
        # start_str_left.velocity = [vel, vel, vel, vel, vel, vel]
        # start_str_left.effort = [effort, effort, effort, effort, effort]

        # start_str_right = JointState()
        # start_str_right.header = Header()
        # start_str_right.header.stamp = rospy.Time.now()
        # start_str_right.name = joints_arr_r

        # Add tripod gait - 3 left 3 right (with contact) & 3 left 3 right (without contact)
        start_str_right = [pose_phase1, pose_phase2, pose_phase1, pose_phase2, pose_phase1, pose_phase2]
        # start_str_right.velocity = [vel, vel, vel, vel, vel, vel]
        # start_str_right.effort = [effort, effort, effort, effort, effort]
        

        # pub.publish(start_str_left)
        # pub.publish(start_str_right)
        for i in range(6):
            pub_l[i].publish(start_str_left[i])
            pub_r[i].publish(start_str_right[i])


        # pose_phase1 , pose_phase2 = swap_phase(pose_phase1, pose_phase2)

        # if delta >= abort_after:
        #     break
