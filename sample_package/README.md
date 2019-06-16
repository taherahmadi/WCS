## Sample Package
this package is contains some sample python code (only python api is available at the moment) and some sample custom .msg files to show how to use WCS.

You will need to create a custom message; let's assume you want to send map data. (in this case there is a predefined msg file in communication_node package: [communication_node/msg/Data_Map.msg](https://github.com/taherahmadi/WCS/blob/master/communication_node/msg/Data_Map.msg))

Map data in ROS is by default of type: nav_msgs/OccupancyGrid
Now we have to create WCS message which carries OccupancyGrid as it's payload.
here is how msg file should look like:

```python
  Header header
  string source
  string destination
  string command
  nav_msgs/OccupancyGrid data 
```

each WCS msg must have:
```
strig source
string destination
```
 it can also have these optional fields:
 ```
  Header header
  string command
 ``` 

the "data" field is the actual payload that can any defiend data type.
remember, in this case we wanted to send map data so we chose nav_msgs/OccupancyGrid as data. 

After creating the msg file inside "./msg/" directory; since the message may have external dependencies (such as nav_msgs/OccupancyGrid in this case) these dependencies should be listed in the CMakelist in "generate_messages" part and in package.xml as "run_depend" and "build depend". 

now build the package to generate the messages . 
