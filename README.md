# WCS
Wireless Communication Simulator Version 0.1

---

## Prerequisites

In order to run the WCS system you need the followings:
* Ubuntu 16.04
* ROS Kinetic
* Gazebo Version 7 

## Setting up the plugin and world file for Gazebo

Gazebo_information_plugins package contains the code to retrieve information from the Gazebo world
and a sample .world file.

to set up this package and prepare your world follow the steps:

* 1- compile the packages with catkin_make

* 2- add (your_workspace)/devel/lib to GAZEBO_PLUGIN_PATH :
...you can do it by adding `export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/(your_workspace)/devel/lib` to your ~/.bashrc or ~/.zshrc

* 3- add libgazebo_world_plugin_v7.so as a plugin to the .world file that you want to use in gazebo:
...you can do it by adding `<plugin name='gazebo_simple_world' filename='libgazebo_simple_world.so'/>` to your .world file

### important
make sure all the models in your .world file start with one of the following strings "Hard_wall","Soft_wall","Medium_wall"



## Preparing .msg files and message handler


* 1- creating message files:

  ...the communication_node package contains code for message_handler node and update_info node and messanger api files and also msg files used by the message_handler.In the msg directory there are several .msg files. Data_sample.msg is an example.

...Each Data_*.msg file has four fields . header,source and destenation are always the same for
...all files. the fourth field is data and can have different types based on your own message files or common ROS messages like odometry or occupancygrid or laserscan .
...for every messeage type you want to use you have to create one Data_*.msg file.

...Next you have to add all Data_*.msg that you want to add_message_files in the CMakeLists.txt of this package 
...if Data_*.msg depends on a package named mypackage then you need to add mypackage to generate_messages in the CMakeLists.txt of this package 
...Data_sample.msg and sample_package and sample_message.msg are good examples.

* 2- setting up message handler:
...message_handler.py in the scripts folder is responsible for transporting message from one node to another.
...before you can use it , you need to set type of the messages that you want to use.
...for each message type you have to add a python list to the `message_list` variable in line 114 of message_handler.py
... the list has 2 elements:

..* the first element is the type of the message 
..* the second element is the tag that should be unique arbitary string for each message type 




## Sending and Receiving messages


* 1- add `from  communication_node.messenger_api import *` and `from communication_node.msg import *` to top of your python code.


* 2- for sending messages use `send_message()` function which takes 3 arguments
..* the first argument is the message object
..* the second argumnet is the type of the message 
..* the third argument is the tag of message that should be equal to the tag used in message_handler.py


* 3- for receiving messages use `receive_message()` which returns a standard ros subscriber object and takes 4 arguments:
..* the first argument is the namespace of the destination node
..* the second argumnet is the type of the message 
..* the third argument is the tag of message that should be equal to the tag used in message_handler.py
..* the fourth argument is the callback function that will be called when a new message has arrived

files in the sample_package/scripts are good examples


## Testing the system

* 1- launch gazebo simulator.

* 2- add your robots to gazebo.

* 3- for each one of your robots rosrun registration_client from communication_node package with the robotname as argument.

* 4- rosrun or launch update_info.py and wait for a several seconds.
  
* 5- rosrun or launch message_handler.py.

* 6- now launch your nodes.



# important 

currently the system has a few issuses that don't make any problems under most circumstances but it's better to be careful 
below is the list of these known issuses :
1. there is an open issuse with communication_node package
2. defining custom messages is not generic enought
3. frequency of update_info.py should be dynamic due to network traffic 
