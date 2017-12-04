#!/usr/bin/env python
# coding=utf-8
"""Message Handler.

# Authors:  Sajjad Azami <sajjadaazami@gmail.com>
# License:  BSD 3 clause

Plays the server role in communication_node

Relations
----------
subscribes from /message_server topic,
publishes on corresponding nodes /ns/message_status topic

"""
import os
import signal
import sys
from time import gmtime,strftime
import rospy
from communication_node.msg import Data_Position
from environment_information import get_object_distance
from propagation_models import one_slope_model_checker

debuger_mode=False
information_logger=None
rate=None

def on_exit(*args):
    global information_logger
    print ( "\n EXITING MESSAGE HANDLER")
    if information_logger!=None :
         information_logger.write("\n ###################### \n ###################### \n")
         information_logger.close()
    sys.exit(0)

def callback(data):
    global information_logger
    # TODO handle for different message types
    # TODO prop_model = data.prop_model
    print("new data received")
    prop_model = '1sm'
    if prop_model == '1sm':
        # distance = get_object_distance("pioneer3at", "Dumpster")
        distance = get_object_distance(data.destination, data.source)
        print type(distance)
        result = one_slope_model_checker(distance=distance)
        if result:
            message_publisher = rospy.Publisher(data.destination + '/inbox', Data_Position, queue_size=10)
            i=0
            while not ( rospy.is_shutdown() or i>1):
                message_publisher.publish(data)
                i+=1
                rate.sleep()
            i=0
            print "communication is possible"
            if debuger_mode==True :
               information_logger.write(data.command+"".join(["-" for k in range(0,8-len(data.command))]))
               information_logger.write(data.source+"".join(["-" for k in range(0,11-len(data.source))]))
               information_logger.write(data.destination+"".join(["-" for k in range(0,16-len(data.destination))]))
               information_logger.write(str(distance)+"".join(["-" for k in range(0,18-len(str(distance)))]))
               information_logger.write("message sent"+"\n")

        else:
            # TODO, ignore the message, send feedback
            if debuger_mode==True :
              information_logger.write(data.command+"".join(["-" for k in range(0,8-len(data.command))]))
              information_logger.write(data.source+"".join(["-" for k in range(0,11-len(data.source))]))
              information_logger.write(data.destination+"".join(["-" for k in range(0,16-len(data.destination))]))
              information_logger.write(str(distance)+"".join(["-" for k in range(0,18-len(str(distance)))]))
              information_logger.write("failed"+"\n")
            print "communication is not possible"


def listener():
    global information_logger
    global rate
    global debuger_mode
    rospy.init_node('communication_node_message_handler')
    rate=rospy.Rate(10)
    debuger_mode=rospy.get_param("debuger_mode",default=False)
    if debuger_mode==True :
         log_file=rospy.get_param("log_file",default="results.txt")
         information_logger =  open("./"+log_file, "a")
         information_logger.write("\n ###################### \n ###################### \n")
         information_logger.write("\n This is the result of test on "+strftime("%Y-%m-%d %H:%M:%S", gmtime()) + " GMT time \n")
         information_logger.write("Number--Source-----Destination-----Distance----------Outcome\n");
    rospy.Subscriber("/message_server", Data_Position, callback,queue_size=50)
    signal.signal(signal.SIGINT, on_exit)
    signal.signal(signal.SIGTERM, on_exit)
    rospy.spin()


listener()
