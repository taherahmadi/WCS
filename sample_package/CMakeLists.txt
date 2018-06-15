cmake_minimum_required(VERSION 2.8.3)
project(sample_package)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   sample_message.msg
 )


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(

  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)




#add_executable(sample_publisher src/sample_publisher.cpp)
#target_link_libraries(sample_publisher ${catkin_LIBRARIES})

#add_executable(sample_subscriber src/sample_subscriber.cpp)
#target_link_libraries(sample_subscriber ${catkin_LIBRARIES})


