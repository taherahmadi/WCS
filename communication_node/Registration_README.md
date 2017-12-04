## WSS Registration Node:
a node for registering robots in WSS communuication node in order to know each other and be able to send queries of data communication to other robots.

## Protocol:
to be adedd...

## Usage
after sourcing environment
run registration server
```
rosrun registration_node registration_server
```
then you can register your robot using this lines in your code:
```
// construct action client
actionlib::SimpleActionClient<registration_node::RegistrationAction> ac("registration", true);

// wait for the action server to start
ac.waitForServer(); //will wait for infinite time

// send a goal to the action
registration_node::RegistrationGoal goal;
goal.robot_namespace = "robot_1";
ac.sendGoal(goal);
```
or modifying and using registration client node:
```
rosrun registration_node registration_client 
```
you can check list of robots registered on parameter server this way:
```
rosparam get /robots_list
```
