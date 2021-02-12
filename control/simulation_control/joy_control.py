#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *


# Receives joystick messages (subscribed to Joy topic)
# then converts the joystick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizontal controls angular speed

def init():
    """
    
    Init robot legs into left and right subarrays

    """

    # Left joints array
    joints_arr_l = []
    # Right joints array
    joints_arr_r = []

    for i in range(6):
        str_l = "/strirus_robot/joint{0}_position_controller_l/command".format(i)
        str_r = "/strirus_robot/joint{0}_position_controller_r/command".format(i)
        joints_arr_l.append(str_l)
        joints_arr_r.append(str_r)

    return joints_arr_l, joints_arr_r


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


def configuration_1(joints_arr_l, joints_arr_r, data):
    """
    Configuration of legs movement with phase difference type 1 in radians

    :param: joints_arr_l, joints_arr_r: joints array that include joints that should move, data - gamepad buttons and sticks axis values
    :return None
    """

    phase_1 = 0.0
    phase_2 = 1.22
    phase_difference = phase_2 - phase_1
    for i in range(len(joints_arr_l)):
        if i % 2 != 0:
            rospy.Publisher(joints_arr_l[i], Float64, queue_size=10).publish(data)
            rospy.Publisher(joints_arr_r[i], Float64, queue_size=10).publish(data)
        else:
            rospy.Publisher(joints_arr_r[i], Float64, queue_size=10).publish(phase_difference * data)
            rospy.Publisher(joints_arr_l[i], Float64, queue_size=10).publish(phase_difference * data)


def configuration_2(joints_arr_l, joints_arr_r, data):
    """
    Configuration of legs movement with phase difference type 2 in radians

    :param: joints_arr_l, joints_arr_r: joints array that include joints that should move, data - gamepad buttons and sticks axis values
    :return None
    """

    phase_1 = 0.45
    phase_2 = -1.20
    phase_difference = phase_1 - phase_2
    for i in range(len(joints_arr_l)):
        if i % 2 != 0:
            rospy.Publisher(joints_arr_l[i], Float64, queue_size=10).publish(phase_difference * data)
            rospy.Publisher(joints_arr_r[i], Float64, queue_size=10).publish(phase_difference * data)
        else:
            rospy.Publisher(joints_arr_r[i], Float64, queue_size=10).publish(data)
            rospy.Publisher(joints_arr_l[i], Float64, queue_size=10).publish(data)


def callback(data):
    """
    Callback function: publish data sended from gamepad sticks with different configuartions that swithces with gamepad buttons
    
    :param data: gamepad axes data
    :return: None
    """

    config_1 = data.buttons[0]
    config_2 = data.buttons[1]

    # l, r = init()

    print("Checked conf # {0}".format(config_1))
    print("Checked conf # {0}".format(config_2))

    l = ['leg_left_5_revolute_joint', 'leg_left_4_revolute_joint', 'leg_left_3_revolute_joint',
         'leg_left_2_revolute_joint', 'leg_left_1_revolute_joint', 'leg_left_0_revolute_joint']
    r = ['leg_right_5_revolute_joint', 'leg_right_4_revolute_joint', 'leg_right_3_revolute_joint',
         'leg_right_2_revolute_joint', 'leg_right_1_revolute_joint', 'leg_right_0_revolute_joint']

    # Put all joints to start configuration
    start_conf(l, r)

    if config_1 == 1:
        rospy.loginfo("START: Using configuration #1")
        configuration_1(l, r, data)
        rospy.loginfo("FINISH: Using configuration #1")

    if config_2 == 1:
        rospy.loginfo("SATRT: Using configuration #2")
        configuration_2(l, r, data)
        rospy.loginfo("FINISH: Using configuration #1")

    for i in range(len(l)):
        rospy.Publisher(l[i], Float64, queue_size=10).publish(data.axes[0] * 3.14)
        rospy.Publisher(r[i], Float64, queue_size=10).publish(data.axes[0] * 3.14)


# Initializes everything
def start():
    """
    
    Start function, init joy node

    """
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)

    # starts the node
    rospy.init_node('Joy2StriRus')
    rospy.spin()


if __name__ == '__main__':
    start()
    init()
