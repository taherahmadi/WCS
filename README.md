# WCS
Wireless Communication Simulator Version 0.1

for running these packages follow the steps:

* 1- compile the packages with catkin_make

* 2- add (your_workspace)/devel/lib to GAZEBO_PLUGIN_PATH :

* 3- add libgazebo_world_plugin_v7.so as a plugin to the .world file that you want to use in gazebo:

* 4- launch gazebo simulator:

* 5- add your robots to gazebo :

* 6- for each one of your robots rosrun registration_client from communication_node package with the robotname as argument:

* 7- launch message_handler:
   '''
   rosrun communication_node message_handler_exprimental.py
   '''
* 8- launch all other nodes:
