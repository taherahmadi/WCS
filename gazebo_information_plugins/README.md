
# IMPORTANT
this package works with gazebo 7

after compiling the package make sure you add the following line to your ~/.bashrc or ~/.zshrc:




`export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/"path to your catkin workspace"/devel/lib`


also add the following line to the .world file inside <world></world> tags:
  `<plugin name='gazebo_connection_plugin' filename='libgazebo_connection_plugin.so'/>`

