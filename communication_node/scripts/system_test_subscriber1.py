#!/usr/bin/env python

import rospy
from messenger_api import send_message
from messenger_api import receive_message
from communication_node.msg import Data_Position


def callback(data):

    print ("new message",data.command)


def main():
    receive_message("robot1", "Data_Position", callback)


if __name__ == "__main__":
    rospy.init_node("test_subscriber1")
    main()
    print("subscriber started")
    rospy.spin()
