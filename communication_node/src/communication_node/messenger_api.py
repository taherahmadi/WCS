#!/usr/bin/env python
# coding=utf-8
"""Messenger API.

# Authors:  Sajjad Azami <sajjadaazami@gmail.com>
#           MohammadHossein GohariNejad <hoseingohari76@gmail.com>
# License:  BSD 3 clause

API for sending and receiving message

Relations
----------
subscribes from corresponding node's (generic)robot_namespace/inbox topic,
publishes on /message_server topic

"""

import rospy
from communication_node.msg import Data_Map, Data_Image, Data_Position



def send_message(message=None, message_type=None,message_tag=""):
    """Sending a message
    This is the message sending protocol

    :parameter
    message : *(message_type).msg type message
    message_type : type, type of the sending message
    message_tag : string, tage for the type of the message

    Relations
    publishes to /message_server_+"message_tag"
    ----------
    """
    message_publisher = rospy.Publisher("/message_server_"+message_tag, message_type, queue_size=10)
    # rospy.init_node(robot_namespace + '_message_sender_node', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    message_publisher.publish(message)
    rate.sleep()




def receive_message(name_space="", message_type=None,message_tag="", callback_function=None):
    """Receive a message
    This is the a wrapper around a ros subscriber

    :parameter
    name_space : your robot's name_space
    message_type : type, type of the message received by this function
    message_tag : string, tag for messagetype
    --------------------
    Returns: the subscriber object
    --------------------
    Relations
    subscribes to /inbox_+"message_tag"
    ----------
    """
    return rospy.Subscriber(name_space + "/inbox_"+message_tag, message_type, callback_function)


