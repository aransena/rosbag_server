#!/usr/bin/env python

import rospy
from rosbag_server_classes import RobotRecordServer

if __name__ == '__main__':
    rospy.init_node('rosbag_server')
    server = RobotRecordServer(rospy.get_name())
    rospy.loginfo("rosbag server ready")
    rospy.spin()
