
below are the instructions you need to work with this system.

firts you need to create system compatable message from your own message files 
in the msg directory there are several .msg files
Data_sample.msg is an example. each Data_*.msg file has four fields . header,source and destenation are always the same for
all files. the fourth field is data and can have different types based on your own message files or common ROS messages like odometry or occupancygric or laserscan .
for every messeage type you want to use you have to create one Data_*.msg file.

next you have to add all Data_*.msg that you want to add_message_files in the CMakeLists.txt of this package 
if Data_*.msg depends on a package named mypackage then you need to add mypackage to generate_messages in the CMakeLists.txt of this package 

now you can compile .


before running the system go to message_handler.py file and on line 157 you find message_list which is a python list of python lists.
for each Data_*.msg that you want to use add a new list to the message_list with the following content ["/message_server_","/inbox_($message_tag)",Data_*,"($message_tag)",None]
for example for Data_Odom we can have ["/message_server_","/inbox_odom",Data_Odom,"odom",None]
the ($message_tag) is an arbitary tag defined by you for every Data_*.msg . there are no restrictions on nameing the tags but you need to have only one tag for each Data_*.msg file and 
keep this tag in mind because we'll need it later.



for sending and recieving messages you need to use to functions from messenger_api.py . 
system_test_publisher1.py and system_test_subscriber1.py are examples of how to use this api.
