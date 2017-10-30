#!/usr/bin/env python

import rospy
import time
import actionlib
import os
from sawyer_learning_interface.msg import RecordingAction, RecordingGoal

def feedback(msg):
    print msg


def recording_client():
    client = actionlib.SimpleActionClient('recording_server', RecordingAction)
    client.wait_for_server()
    goal = RecordingGoal(save_folder=os.path.join(os.getenv("HOME"), "output"), duration=3, save_name="3")
    client.send_goal(goal, feedback_cb=feedback)
    # #
    time.sleep(5)
    client.cancel_goal()
    client.wait_for_result()
    result = client.get_result()

    print "Result: ", result

    return result


if __name__=='__main__':
    try:
        rospy.init_node('recording_test_client')
        result = recording_client()
        print "Result: ", result
    except rospy.ROSInterruptException:
        print "Program interrupted"
