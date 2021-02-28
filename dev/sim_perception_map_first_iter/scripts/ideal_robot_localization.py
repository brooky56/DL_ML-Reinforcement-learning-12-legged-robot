#!/usr/bin/env python
"""Take data from gazebo and make a TF transform from gazebo to robot baselink"""

import rospy
from geometry_msgs.msg import Pose, TransformStamped
import tf_conversions
import tf2_ros


def callback(data):
    try:
        handle_strirus_pose(data)
    except ValueError:
        rospy.logerr(
            "There is no model called " + (rospy.get_param('/ideal_robot_localization/robot_name', "strirus_gamma") +
                                           ", shutdown the node"))
        rospy.signal_shutdown("meow")
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def handle_strirus_pose(strirus_pose):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = rospy.get_param(
        '/ideal_robot_localization/world_frame', "world")
    t.child_frame_id = rospy.get_param(
        '/ideal_robot_localization/robot_base_frame', "base_footprint")
    t.transform.translation = strirus_pose.position
    t.transform.rotation = strirus_pose.orientation
    br.sendTransform(t)


if __name__ == '__main__':
    try:
        # log_level is needed for present debugging info
        if rospy.get_param('/ideal_robot_localization/debug', "true"):
            logging = rospy.DEBUG
        else:
            logging = rospy.INFO
        rospy.init_node('ideal_robot_localization',
                        anonymous=True, log_level=logging)
        rospy.Subscriber("/coppeliasim/robot_state", Pose, callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
