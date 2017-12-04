
* 1- run the gazebo
* 2- run registration_server.cpp :
	```
	rosrun communication_node registration_server
	```
* 3- for every robot in the gazebo launch registration_client with the name of robot as arg:
  ```
  rosrun communication_node registration_client -arg "robot_name"
  ```
* 3- launch the message_handler:
  ```
  rosrun communication_node message_handler.py
  ```
