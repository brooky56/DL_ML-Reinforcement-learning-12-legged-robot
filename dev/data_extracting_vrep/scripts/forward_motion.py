#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import numpy as np
import time

if __name__ == '__main__':
    pub_l = []
    pub_r = []
    for i in range(6):
        str_l = "/strirus_robot/joint{0}_position_controller_l/command".format(i)
        str_r = "/strirus_robot/joint{0}_position_controller_r/command".format(i)
        pub_l.append(rospy.Publisher(str_l, Float64, queue_size=10))
        pub_r.append(rospy.Publisher(str_r, Float64, queue_size=10))
    rospy.init_node('forward_motion')
    # rate = rospy.Rate(50)

    print("Enter desired position:")
    pos = float(input())

    pose_phase1 = rospy.get_param('/leg/shape_dir', 1) * pos
    pose_phase2 = rospy.get_param('/leg/shape_dir', 1) * (pos + np.pi)

    while not rospy.is_shutdown():
        start_str_left = [pose_phase2, pose_phase1, pose_phase2, pose_phase1, pose_phase2, pose_phase1]
        start_str_right = [pose_phase1, pose_phase2, pose_phase1, pose_phase2, pose_phase1, pose_phase2]
        for i in range(6):
            pub_l[i].publish(start_str_left[i])
            pub_r[i].publish(start_str_right[i])
