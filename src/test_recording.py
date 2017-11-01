#!/usr/bin/env python

import rospy
import time
import actionlib
import os

from rosbag_server.msg import RecordingAction, RecordingGoal

def feedback(msg):
    print msg.status.split(',')


def recording_client():
    client = actionlib.SimpleActionClient('recording_server', RecordingAction)
    client.wait_for_server()
    goal = RecordingGoal(save_folder=os.path.join(os.getenv("HOME"), "output"),
                         duration=3,
                         save_name="",
                         topics=['/topic1', '/topic2'],
                         args=['--size=10'])
    client.send_goal(goal, feedback_cb=feedback)

    time.sleep(5)
    client.cancel_goal()
    client.wait_for_result()
    _result = client.get_result()

    return _result


if __name__=='__main__':
    try:
        rospy.init_node('recording_test_client')
        result = recording_client()
        print result.outcome.split(',')
    except rospy.ROSInterruptException:
        print "Program interrupted"
