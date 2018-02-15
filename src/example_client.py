#!/usr/bin/env python
# Press esc or ctrl key to stop program
# Depends on pynput:
# pip install pynput

import rospy
import threading

from pynput import keyboard
from rosbag_server_classes import RobotRecordClient

global client


def feedback(msg):
    print msg.status.split(',')


def on_press(key):
    global client
    if key == keyboard.Key.ctrl:
        print "stop pressed"
        client.stop()
        return False
    elif key == keyboard.Key.esc:
        print "stop pressed"
        client.stop()
        return False  # Stop key listener


def key_listener():
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()


if __name__=='__main__':
    global client
    rospy.init_node('recording_test_client')
    client = RobotRecordClient('rosbag_server')
    t = threading.Thread(target=key_listener)
    t.start()
    keyboard_control = keyboard.Controller()
    try:

        result = client.record("./", "test", ["/robot/limb/right/endpoint_state"], feedback_callback=feedback)

        print result.outcome.split(',')
        keyboard_control.press(keyboard.Key.esc)

    except rospy.ROSInterruptException:
        print "Program interrupted"

