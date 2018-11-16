#!/usr/bin/env python

import rospy
from rosbagServer import RosbagServer

if __name__ == '__main__':
    rospy.init_node('rosbag_server')
    server = RosbagServer(rospy.get_name(), log_interval=5)
    rospy.loginfo("rosbag server ready")
    rospy.spin()
