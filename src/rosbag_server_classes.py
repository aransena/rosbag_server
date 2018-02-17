#!/usr/bin/env python

import rospy
import actionlib
import time
import datetime
import psutil

import subprocess
import os
import signal

from rosbag_server.msg import RecordingAction, RecordingResult, RecordingFeedback, RecordingGoal

def terminate_process_and_children(p):
    process = psutil.Process(p.pid)
    for sub_process in process.get_children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    p.wait()


class RobotRecordServer(object):
    _feedback = RecordingFeedback()
    _result = RecordingResult()

    def __init__(self, name='rosbag_server'):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                RecordingAction,
                                                execute_cb=self.execute_cb,
                                                auto_start=False)
        self._as.register_preempt_callback(self.preempt_cb)
        self._home_folder = os.getenv("HOME")
        rospy.loginfo("rosbag server running")
        self._as.start()

    def preempt_cb(self):
        pass

    def execute_cb(self, goal):
        wait_rate = rospy.Rate(10)
        success = True
        if goal.save_name == "":
            bagname = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        else:
            bagname = goal.save_name

        if ".bag" not in bagname:
            bagname += ".bag"

        if goal.save_folder == "":
            bagfolder = self._home_folder
        else:
            bagfolder = goal.save_folder

        bagpath = os.path.join(bagfolder, bagname)

        self._feedback.status = "setup,file," + bagpath
        self._as.publish_feedback(self._feedback)

        if not os.path.os.path.isdir(bagfolder):
            os.makedirs(bagfolder)

        if os.path.isfile(bagpath):
            self._result.outcome = "error,exists," + bagname + " already exists in " + bagfolder
            self._as.set_aborted(self._result)

        else:

            cmd = "rosbag record"

            for topic in goal.topics:
                cmd += " " + topic

            cmd += " -O " + bagname

            for arg in goal.args:
                cmd += " " + arg

            bag_recorder = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                            shell=True,
                                            cwd=bagfolder,
                                            preexec_fn=os.setsid)

            self._feedback.status = "start,command," + cmd
            self._as.publish_feedback(self._feedback)

            start_time = time.time()
            elapsed_time = start_time
            logged = False
            while not self._as.is_preempt_requested():
                elapsed_time = time.time() - start_time

                log_interval = 5.0
                if int(elapsed_time) % log_interval == 0 and not logged:
                    self._feedback.status = "recording,time," + str(round(elapsed_time, 2)) + ",sec"
                    self._as.publish_feedback(self._feedback)
                    logged = True
                elif int(elapsed_time) % log_interval != 0 and logged:
                    logged = False

                if self._as.is_preempt_requested():
                    break
                wait_rate.sleep()

            pid = bag_recorder.pid
            os.killpg(pid, signal.SIGINT)

            self._feedback.status = "stop,time," + str(round(elapsed_time, 2)) + ",sec"
            self._as.publish_feedback(self._feedback)

            if not success:
                self._result.outcome = "error,"
                cmd = "rm *.active"
                try:
                    subprocess.call(cmd, shell=True, cwd=bagfolder)
                except Exception as e:
                    print e, e.message

            else:
                self._result.outcome = "success,path," + bagpath
                rospy.loginfo('%s: Succeeded' % self._action_name)

            self._as.set_preempted(self._result)


class RobotRecordClient(object):

    def __init__(self, name='rosbag_server'):
        self._client = None
        self._name = name
        self._result = None
        self._goal = None
        self._result = None
        self._connect_to_server()

    def _connect_to_server(self):
        self._client = actionlib.SimpleActionClient(self._name, RecordingAction)
        self._client.wait_for_server()

    def _set_goal(self, savefolder, savename, topic_list, rosbag_args_list=[]):
        self._goal = RecordingGoal(save_folder=savefolder,
                                   save_name=savename,  # save_name="bagname" OR save_name="bagname.bag"
                                   topics=topic_list,  # topic_list=['/topic1','/topic2']
                                   args=rosbag_args_list)  # arg_list=['arg1','arg2']

    def _send_goal(self, feedback_callback = None):
        self._client.send_goal(self._goal, feedback_cb=feedback_callback)
        self._client.wait_for_result()
        result = self._client.get_result()
        return result

    def record(self, savefolder, savename, topic_list, rosbag_args_list=[], feedback_callback=None):  #, postrecord_fn=None,postrecord_fn_args=None):
        self._set_goal(savefolder, savename, topic_list, rosbag_args_list)
        self._result = self._send_goal(feedback_callback)
        # if postrecord_fn:
        #     if postrecord_fn_args:
        #         postrecord_fn(postrecord_fn_args)
        #     else:
        #         postrecord_fn
        return self._result

    def stop(self):
        self._client.cancel_goal()



