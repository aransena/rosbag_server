import actionlib

from rosbag_server.msg import RecordingAction, RecordingResult, RecordingFeedback, RecordingGoal

class RosbagClient(object):

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

    def record(self, savefolder, savename, topic_list, rosbag_args_list=[], feedback_callback=None):
        self._set_goal(savefolder, savename, topic_list, rosbag_args_list)
        self._result = self._send_goal(feedback_callback)

        return self._result

    def stop(self):
        self._client.cancel_goal()



