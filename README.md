# rosbag_server
This package provides an actionlib interface to rosbag. The client is able to send a rosbag record command to the server with their selection of topics and args, and can receive basic status updates from the server. The client also has the option of interrupting and cancelling the recording action.

Example cilent in scripts/

## Starting the server
To start the server, run:
```
rosrun rosbag_server start_server.py
```

## Initialising the client
To interact with the server, define a client in your code
```python
from rosbagClient import RosbagClient
...
client = RosbagClient('rosbag_server')
```

## Recording topics and getting feedback
To record topics, send a request to the server using the record function. rosbag args and feedback callback functions are optional.
```python
def record(self, savefolder, savename, topic_list, rosbag_args_list=[], feedback_callback=None)
```

Note that the provided save folder and name must be unique - the server will return an error status if the requested bag path already exists.

Example client setup:
```python
result = client.record("./", "test", ["/test10", "/test100", "/test1000"],[-l 100], feedback_callback=feedback)
```

Example feedback function:
```python
def feedback(msg):
    print msg.status.split(',')
```


Example output with client printing feedback messages as defined above, recording from three test topics publishing Empty messages at 10, 100, and 1k Hz.

Server reports it is setting up to record with the path ./test.bag
```
['setup', 'path', '<your path to the bag data folder>/test.bag']
```

Server reports the command sent to it in full
```
['start', 'command', 'rosbag record /test10 /test100 /test1000 -O test.bag -l 100']
```

Server provides updates on recording status
```
['recording', 'elapsed_time', '0.0']
['recording', 'elapsed_time', '5.09']
['recording', 'elapsed_time', '10.09']
['stop', 'elapsed_time', '10.79']
['success', 'path', '<your path to the bag data folder>/test.bag']
```

The client can be cancelled at any point after recording starts using
```
client.stop()
```
however the server should clean up any threads if recording stops naturally (e.g. after 100 messages recorded on each topic in the example above).
