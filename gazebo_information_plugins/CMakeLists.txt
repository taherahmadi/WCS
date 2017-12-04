cmake_minimum_required(VERSION 2.8.3)
project(gazebo_information_plugins)  # --- ${PROJECT_NAME} == gazebo_information_plugins

# note that while this packge contains gazebo plugins which are ros aware , these plugins should
#be used by being attached to gazebo models or worlds or etc. like simple gazebo plugins

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  message_generation
  std_msgs
  gazebo_ros
  gazebo_plugins
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

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


add_service_files(
  FILES
  distance_serivce.srv
)
generate_messages(
  DEPENDENCIES
  nav_msgs
  std_msgs
)

catkin_package(
  DEPENDS
    roscpp
    rospy
    nav_msgs
    message_generation
    std_msgs
    gazebo_ros
    gazebo_plugins
    INCLUDE_DIRS
    LIBRARIES
)




###########
## Build ##
###########



###### for gazebo 7
add_library(gazebo_simple_world src/SimpleWorldPlugin_v7.cpp)
target_link_libraries(gazebo_simple_world ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


add_library(gazebo_plugin_contact src/ContactPlugin.cpp)
target_link_libraries(gazebo_plugin_contact ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
