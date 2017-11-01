#!/usr/bin/env python

import rospy
import actionlib
import time
import datetime
import psutil

import subprocess, os, signal

from rosbag_server.msg import RecordingAction, RecordingResult, RecordingFeedback


def terminate_process_and_children(p):
    process = psutil.Process(p.pid)
    for sub_process in process.get_children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    p.wait()


class RobotRecordServer(object):
    _feedback = RecordingFeedback()
    _result = RecordingResult()

    def __init__(self, name, testing=False):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                RecordingAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.register_preempt_callback(self.preempt_cb)
        self._testing = testing
        self._home_folder = os.getenv("HOME")
        rospy.loginfo("rosbag Server Running")
        self._as.start()

    def preempt_cb(self):
        pass

    def execute_cb(self, goal):
        wait_rate = rospy.Rate(10)
        success = True
        print "GOAL topics: ", goal.topics
        if goal.save_name == "":
            bagname = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            self._feedback.status = "status: no_filename:"+bagname+".bag"
            self._as.publish_feedback(self._feedback)
        else:
            bagname = goal.save_name

        if goal.save_folder == "":
            bagfolder = self._home_folder
        else:
            bagfolder = goal.save_folder

        bagpath = os.path.join(bagfolder, bagname+".bag")

        if os.path.isfile(bagpath):
            self._result.outcome = "error: " + bagname + ".bag already exists in " + bagfolder
            self._as.set_aborted(self._result)

        else:

            if self._testing:
                test_bag_folder = os.path.join(self._home_folder, "bagfiles")
                test_bag = "2017-10-27-14-20-57.bag"
                cmd = "rosbag play " + test_bag
                test_playback = subprocess.Popen(cmd, stdin=subprocess.PIPE, shell=True, cwd=test_bag_folder)


            cmd = "rosbag record "


            for topic in goal.topics:
                cmd += topic + " "

            for arg in goal.args:
                cmd += arg + " "

            print "CMD: ", cmd

            cmd += "-O " + bagname + ".bag"

            print cmd
            bag_recorder = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                            stdout=subprocess.PIPE, shell=True, cwd=bagfolder)
            time.sleep(0.1)
            self._feedback.status = "status: Recording started with command [" + cmd + "]"
            self._as.publish_feedback(self._feedback)

            start_time = time.time()
            logged = False
            while not self._as.is_preempt_requested():
                elapsed_time = time.time() - start_time

                log_interval = 5.0
                if int(elapsed_time) % log_interval == 0 and not logged:
                    self._feedback.status = "status: recording time " + str(round(elapsed_time, 2)) + " sec"
                    self._as.publish_feedback(self._feedback)
                    logged = True
                elif int(elapsed_time) % log_interval != 0 and logged:
                    logged = False

                if self._as.is_preempt_requested():
                    break
                wait_rate.sleep()

            bag_recorder.send_signal(subprocess.signal.SIGINT)

            if self._testing:
                terminate_process_and_children(test_playback)

            self._feedback.status = "Recording Stopped"
            self._as.publish_feedback(self._feedback)

            if success:
                cmds = ["rosbag reindex *.active", "rm *.orig.*", "rename 's/.active//' *"]
                for cmd in cmds:
                    try:
                        subprocess.call(cmd, shell=True, cwd=bagfolder)
                    except Exception as e:
                        print e, e.message
                        pass

            if not success:
                self._result.outcome ="error: "
                cmd = "rm *.active"
                try:
                    subprocess.call(cmd, shell=True, cwd=bagfolder)
                except Exception as e:
                    print e, e.message

            else:
                self._result.outcome = bagpath
                rospy.loginfo('%s: Succeeded' % self._action_name)

            self._as.set_preempted(self._result)


if __name__=='__main__':
    rospy.init_node('recording_server')
    server = RobotRecordServer(rospy.get_name(), testing=False)
    rospy.spin()
