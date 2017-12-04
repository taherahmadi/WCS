#!/usr/bin/env python

import rospy
from messenger_api import send_message
from messenger_api import register
from communication_node.msg import Data_Position


def main():
    msg = Data_Position()
    msg.source = "robot1"
    msg.destination = "robot2"
    rate = rospy.Rate(1)
    i = 0
    while not rospy.is_shutdown():
        i = i + 1
        msg.command = str(i)
        send_message(msg,'Data_Position')
        rospy.loginfo("sent number %d", i)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("test_publisher1")
    main()
    rospy.spin()
