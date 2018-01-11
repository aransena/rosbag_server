#!/usr/bin/env python

import rospy
import actionlib
import os
import threading
import intera_interface
import time

from pynput import keyboard
from rosbag_server.msg import RecordingAction, RecordingGoal


global client


def feedback(msg):
    print msg.status.split(',')


def recording_client(filename):
    global client
    client = actionlib.SimpleActionClient('recording_server', RecordingAction)
    client.wait_for_server()
    goal = RecordingGoal(save_folder=os.path.join(os.getenv("HOME"), "tray_data_ver2", "grid"),
                         save_name="goal", # save_name="bagname" OR save_name="bagname.bag"
                         topics=['/robot/limb/right/endpoint_state', '/robot/joint_states', '/hand_control'], # topics=['/topic1','/topic2']
                         args=['-l', '1']) # args=['arg1','arg2']

    client.send_goal(goal, feedback_cb=feedback)
    client.wait_for_result()
    _result = client.get_result()
    return _result


def on_press(key):
    global client
    if key == keyboard.Key.ctrl:
        print "stop pressed"
        client.cancel_goal()
        return False
    elif key == keyboard.Key.esc:
        # Stop listener
        return False


def key_listener():
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()


if __name__=='__main__':
    global client

    filename = "place_goal"
    # if "a" in filename:
    #     neutral_start = True
    # else:
    #     neutral_start = False
    neutral_start = False
    rospy.init_node('recording_test_client')

    if neutral_start:
        intera_interface.RobotEnable().enable()
        limb = intera_interface.Limb('right')
        limb.move_to_neutral()
        time.sleep(1)

    client = None
    t = threading.Thread(target=key_listener)
    t.start()
    keyboard_control = keyboard.Controller()

    try:
        result = recording_client(filename)
        print result.outcome.split(',')
        keyboard_control.press(keyboard.Key.esc)

    except rospy.ROSInterruptException:
        print "Program interrupted"

