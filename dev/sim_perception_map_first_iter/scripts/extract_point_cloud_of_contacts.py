#!/usr/bin/env python3
"""listen data from contact sensor and extract one point"""

import rospy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud
import message_filters

br = rospy.Publisher(rospy.get_param('/extract_point_cloud_of_contacts/publisher_topic', "/all_legs_contact_points"),
                     PointCloud, queue_size=10)


def callback(d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12):
    data_col = [d1, d2, d3, d4, d5, d6, d7, d8, d9, d10, d11, d12]
    contact_points = []
    for data in data_col:
        if len(data.states):
            for i in data.states:
                # for reducing amount of data
                # for p in i.contact_positions:
                #     contact_points.append(p)
                contact_points.append(i.contact_positions[0])
    if len(contact_points):
        rospy.logdebug("len of points " + str(len(contact_points)))
        handle_contact_points(contact_points)
    else:
        rospy.logdebug("no_data ")


def handle_contact_points(contact_points):
    global br
    t = PointCloud()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = rospy.get_param('/extract_point_cloud_of_contacts/world_frame', "world")
    t.points = contact_points
    br.publish(t)


if __name__ == '__main__':
    try:
        # log_level is needed for present debugging info
        if rospy.get_param('/extract_point_cloud_of_contacts/debug', "true"):
            logging = rospy.DEBUG
        else:
            logging = rospy.INFO
        rospy.init_node('extract_point_cloud_of_contacts', anonymous=True, log_level=logging)
        subs = []
        for side in ["left", "right"]:
            for i in range(6):
                subs.append(message_filters.Subscriber("/leg_" + side + "_" + str(i) + "_bumper", ContactsState))
        ts = message_filters.TimeSynchronizer(subs, 10)
        ts.registerCallback(callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
