#!/usr/bin/env python

import rospy
from messenger_api import *
from communication_node.msg import *
from sample_packge.msg import *

a=None
def callback(data):

    print ("new message",data.data.a,data.data.b,data.data.c)

def main():
    global a;
    a=receive_message("robot2", Data_sample, "sample",callback)


if __name__ == "__main__":
    rospy.init_node("test_subscriber1")
    main()
    print("subscriber started")
    rospy.spin()
