#!/usr/bin/env python

import rospy
from messenger_api import *
from communication_node.msg import *
from sample_packge.msg import *


def main():
    msg = Data_sample()
    msg.source = "robot1"
    msg.destination = "robot2"
    rate = rospy.Rate(1)
    i = 0
    while not rospy.is_shutdown():
        i = i + 1
        temp_var=sample_message();
        temp_var.a=i
        temp_var.b=i*2
        temp_var.c=str(i)+"--"+str(i*2)
        msg.data = temp_var
        send_message(msg,Data_sample,"sample")
        rospy.loginfo("sent message number %d", i)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("test_publisher1")
    main()
    rospy.spin()
