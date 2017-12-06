# CDR message transport type.
#
# Used to transport CDR message via ROS topic
#
#

std_msgs/Header header
uint8 topic_id
uint8 len
uint8[] buffer
