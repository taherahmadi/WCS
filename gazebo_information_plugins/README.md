
IMPORTANT
this package works with gazebo 7






export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/workspaces/communication_node/devel/lib  
 note that communication_node is a catkin workspace

""""""""""



-------------
also add this to the world file
  <plugin name='sosvr_gazebo_plugin_simple' filename='libsosvr_gazebo_plugin_simple.so'/>
  ----------------------------------------------------------

