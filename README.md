# WCS
Wireless Communication Simulator Version 0.1

---
The goal of this project is to provide robot agents in the ROS/Gazebo simulation with an API to transmit data over a simulated wireless network. 
This project consists of two main parts:
1. A Gazebo Plugin
2. A ROS Library

so in order to use this package, you need to 
1. Run Gazebo with the plugin.
2. Create a message for the data type you want to transmit using WCS.
3. Import the library and use the WCS API for transmitting data instead of ROS topic or service.  

Here is the instruction for doing the above steps:

## Prerequisites

In order to run the WCS system you need the followings:
* Ubuntu 16.04
* ROS Kinetic
* Gazebo Version 7 


## Step 1: Setting up the plugin and world file for Gazebo

Gazebo_information_plugins package contains the code to retrieve information from the Gazebo world
and a sample .world file.

to set up this package and prepare your world, follow these steps:

* 1- compile the packages with catkin_make

* 2- add (your_workspace)/devel/lib to GAZEBO_PLUGIN_PATH :

  you can do it by adding `export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/(your_workspace)/devel/lib` to your ~/.bashrc or ~/.zshrc

* 3- add libgazebo_connection_plugin.so as a plugin to the .world file that you want to use in gazebo:

  you can do it by adding `<plugin name='gazebo_connection_plugin' filename='libgazebo_connection_plugin.so'/>` to your .world file inside `<world></world>`tags

### important
It is recommended that the models in your .world file start with one of the following strings "Hard_wall", "Soft_wall", "Medium_wall" if the name of a model does not start with one of the mentioned strings that model will be considered a Soft_wall when computing the signal decay.



## Step 2: Preparing Custom message files and message handler

Before using WCS you need to create WCS compatible messages:
* 1- creating message files:

  the communication_node package contains code for message_handler node and update_info node and messenger API files and also msg files used by the message_handler.In the msg directory, there are several .msg files. Data_Sample.msg is an example.

  Each Data_*.msg file has four fields that the sender has to complete:
     * header: you can use it for time stamp and message number
     * source: a string with the namespace of the sender node.
     * destination: a string with the namespace of the receiving node. 
     * data: this field can have different types based on your own message files or common ROS messages like odometry or occupancygrid or laserscan .
  for every message type you want to use you have to create one Data_*.msg file.

  Next, you have to add all Data_*.msg that you want to add_message_files in the CMakeLists.txt of this package if Data_*.msg depends on a package named mypackage then you need to add mypackage to generate_messages in the CMakeLists.txt of this package.
  
  Data_Sample.msg in communication_node and sample_message.msg in sample_package are good examples.

  Example:
    So let’s say we want to create a WCS message to send Occupancygrid data type.
    First we create Data_Map.msg inside communication_node/msg folder with the following fields:
    ```
      Header header
      string source
      string destination
      string command
      nav_msgs/OccupancyGrid data 
    ```
    
    now since this msg file depends on nav_msgs package, we need to add nav_msgs as a dependency in CMakeLists.txt and package.xml inside the communication_node.
    First, we modify CMakeLists.txt:
    1- find_package(catkin REQUIRED COMPONENTS ) section must contain nav_msgs like the code below:
    ```
    find_package(catkin REQUIRED COMPONENTS
      nav_msgs
      actionlib
      actionlib_msgs
      message_generation
      roscpp
      rospy
      sensor_msgs
      std_msgs
    )
    ```

    2- we also need to add Data_Map.msg in add_message_files(FILES ) section like code below:
    ```
    add_message_files(FILES
      Data_Map.msg
      Data_Image.msg
      Data_Sample.msg
      Data_Odom.msg
      Data_Goal.msg
    )
    ```

    3- we also need to add nav_msgs in generate_messages( DEPENDENCIES ) section:
    ```
    generate_messages(
      DEPENDENCIES
      nav_msgs actionlib_msgs std_msgs sensor_msgs sample_package
    )
    ```

    Now we modify package.xml:
    We just need to add the two following lines to package.xml 
    ```
      <run_depend>nav_msgs</run_depend>
      <build_depend>nav_msgs</build_depend>
    ```
    Now you can build the communication_node package and have the Data_Map.msg file compiled and ready to use.


* 2- setting up message handler:

  message_handler.py in the scripts folder is responsible for transporting messege from one node to another. 
  Before you can use it, you need to set the type of the messages that you want to use. 
  So first make sure the following line is at the top of message_handler.py
  ```
  from communication_node.msg import *
  ```

  for each message type you have to add a python list to the `message_list` variable in line 114 of message_handler.py
  the list has 2 elements:

  * the first element is the type of the message 
  * the second element is the tag that should be a unique arbitrary string for each message type 

  Here we modify it like:
  ```
    message_list=[[Data_Map,"map"],[Data_Sample,"sample"]]
  ```


## Step 3: Sending and Receiving messages
For sending and receiving WCS messages in python we are going to use two sample python codes in sample_package called system_test_publisher and system_test_subscriber.

* 1- add `from  communication_node.messenger_api import *` and `from communication_node.msg import *` to top of your python code.
```
  from  communication_node.messenger_api import *
  from communication_node.msg import *
```

* 2- for sending messages use `send_message()` function which takes 3 arguments
  * the first argument is the message object
  * the second argument is the type of the message 
  * the third argument is the tag of messege that should be equal to the tag used in message_handler.py

we set the tag to “map” in message_handler.py so we are going to do the same here:
```
  message(msg,Data_Map,"map")
```
we need to create a Data_Map message object and fill the “source”, ”destination” and “data” fields. 

* 3- for receiving messages use `receive_message()` which returns a standard ROS subscriber object and takes 4 arguments:
  * the first argument is the namespace of the destination node
  * the second argumnet is the type of the message 
  * the third argument is the tag of message that should be equal to the tag used in message_handler.py
  * the fourth argument is the callback function that will be called when a new message has arrived

```
  a=receive_message("robot0", Data_Map, "map",callback)
```

files in the sample_package/scripts are good examples


## Testing the system

* 1- launch gazebo simulator.

* 2- add your robots to gazebo.

* 3- for each one of your robots rosrun registration_client from communication_node package with the robotname as an argument.

* 4- rosrun or launch update_info.py and wait for a several seconds.
  
* 5- rosrun or launch message_handler.py.

* 6- now launch your nodes.

### for a demo please read Demo.md in package "setup"

### Known issues 
currently, there are a few issues that has no major effect in the performance of the system, but need to get fixed.
1. there is an open issuse with communication_node package
2. defining custom messages is not generic enough
3. frequency of update_info.py should be dynamic due to network traffic 
