#!/usr/bin/env python
'''
Start some test topics:

rostopic pub /test10 std_msgs/Empty -r 10
rostopic pub /test100 std_msgs/Empty -r 100
rostopic pub /test1000 std_msgs/Empty -r 1000
'''

import rospy
import os

from rosbagClient import RosbagClient

# Feedback callback
def feedback(msg):
    print msg.status.split(',')


if __name__ == '__main__':
    rospy.init_node('recording_test_client')
    client = RosbagClient('rosbag_server')
    
    try:
        home_folder = os.path.expanduser('~')
        result = client.record(home_folder+'/rosbag_server_data', "test", ["/test10", "/test100", "/test1000"], ['-l 100'],
                               feedback_callback=feedback)

        print result.outcome.split(',')

    except Exception as e:
        print "Error: ", e
