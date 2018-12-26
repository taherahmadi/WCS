#!/usr/bin/env python

import rospy
from  communication_node.messenger_api import *
from communication_node.msg import *
from sample_package.msg import *

a=None
def callback(data):

    print ("new message",data.data.a,data.data.b,data.data.c)

def main():
    global a
    a=receive_message("robot0", Data_Sample, "sample",callback)


if __name__ == "__main__":
    rospy.init_node("test_subscriber1")
    main()
    print("subscriber started")
    rospy.spin()
