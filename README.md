# rosbag_server

## Goal definition
string save_folder
string save_name
string[] topics
string[] args

## Result definition
string outcome

## Feedback message
string status

# Example
result = client.record("./", "test", ["/topic1", "/topic2"], feedback_callback=feedback)
