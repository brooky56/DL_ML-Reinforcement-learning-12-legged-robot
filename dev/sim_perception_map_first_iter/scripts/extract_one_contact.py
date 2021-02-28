#!/usr/bin/env python3
"""listen data from contact sensor and extract one point"""

import rospy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud

br = rospy.Publisher(rospy.get_param('/extract_one_contact/publisher_topic', "/leg_left_0_contact_points"), PointCloud,
                     queue_size=10)


def callback(data):
    if len(data.states):
        contact_points = []
        for i in data.states:
            for p in i.contact_positions:
                contact_points.append(p)
        rospy.logdebug("len of points " + str(len(contact_points)))
        # handle_contact_points(contact_points)
    else:
        rospy.logdebug("no_data ")

    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def handle_contact_points(contact_points):
    global br
    t = PointCloud()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = rospy.get_param('/extract_one_contact/world_frame', "world")
    t.points = contact_points
    br.publish(t)


if __name__ == '__main__':
    # log_level is needed for present debugging info
    try:
        if rospy.get_param('/extract_one_contact/debug', "true"):
            logging = rospy.DEBUG
        else:
            logging = rospy.INFO
        rospy.init_node('extract_one_contact', anonymous=True, log_level=logging)

        rospy.Subscriber(rospy.get_param('/extract_one_contact/listener_topic', "/leg_left_0_bumper"), ContactsState,
                         callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
