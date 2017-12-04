#IMPORTANT
 first delet the gazebo_information_plugins package 
 and then rename this package to gazebo_information_plugins
 after that you can catkin_make the package and compile it






export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/workspaces/communication_node/devel/lib  
 note that communication_node is a catkin workspace

""""""""""



-------------
also add this to the world file
  <plugin name='sosvr_gazebo_plugin_simple' filename='libsosvr_gazebo_plugin_simple.so'/>
  ----------------------------------------------------------

