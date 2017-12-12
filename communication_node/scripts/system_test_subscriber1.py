#!/usr/bin/env python

import rospy
from messenger_api import *
from communication_node.msg import Data_Position

a=None
def callback(data):

    print ("new message",data.command)

def main():
    global a;
    a=receive_message("robot2", Data_Position, "pose",callback)


if __name__ == "__main__":
    rospy.init_node("test_subscriber1")
    main()
    print("subscriber started")
    rospy.spin()
