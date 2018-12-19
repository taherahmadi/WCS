cmake_minimum_required(VERSION 2.8.3)
project(setup)
find_package(catkin REQUIRED COMPONENTS roscpp rospy nav_msgs tf geometry_msgs sensor_msgs std_msgs)

include_directories(include ${catkin_INCLUDE_DIRS})


## Find OGRE and OGRE-Paging 
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
endif()
include_directories(${OGRE_INCLUDE_DIRS} ${OGRE-Paging_INCLUDE_DIRS})
link_directories(${OGRE_LIBRARY_DIRS} ${OGRE-Paging_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})


###################################
## catkin specific configuration ##
###################################

catkin_package(
 CATKIN_DEPENDS
 nav_msgs
 roscpp
 rospy
 tf
 sensor_msgs
 std_msgs
 geometry_msgs
 INCLUDE_DIRS 
 LIBRARIES
)

###########
## Build ##
###########
