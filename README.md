# rosbag_server
## Starting the server
To start the server, run:
```
rosrun rosbag_server start_server.py
```

## Initialising the client
To interact with the server, define a client in your code
```python
from rosbag_server_classes import RobotRecordClient
...
client = RobotRecordClient('rosbag_server')
```

## Recording topics and getting feedback
To record topics, send a requests to the server using the record function. rosbag args and feedback callback functions are optional.

Note that the provided save folder and name must be unique, the server will return an error status if the requested bag path already exists.
```python
def record(self, savefolder, savename, topic_list, rosbag_args_list=[], feedback_callback=None)
```

```python
def feedback(msg):
    print msg.status.split(',')
```

```python
result = client.record("./", "test", ["/topic1", "/topic2"], feedback_callback=feedback)
```

Example output with client printing feedback messages as defined above

Server reports it is setting up to record to the path ./test.bag
```
['setup', 'path', './test.bag']
```

Server reports the command sent to it in full
```
['start', 'command', 'rosbag record /test10 /test100 /test1000 -O test.bag --duration=10']
```

Server provides updates on recording status
```
['recording', 'elapsed_time', '0.0']
['recording', 'elapsed_time', '5.09']
['recording', 'elapsed_time', '10.09']
['stop', 'elapsed_time', '10.69']
['success', 'path', './test.bag']
```
