to run a test do as i say

first roslaunch setup gserver.launch
second rosrun communication_node registration_server
third roslaunch communication_node spawn.launch
wait a few seconds
fourth roslaunch communication_node spawn2.launch
fifth roslaunch communication_node message_handler.launch also note that you can change the file name for the log file and on or off debuger_mode
sixth roslaunch communication_node test.launch
seventh change the posisition of robots and done you are
