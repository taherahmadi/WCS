#!/usr/bin/env python


"""
Testing the communication_node
"""
import rospy
from messenger_api import send_message
from messenger_api import register
from communication_node.msg import Data_Position

robot_namespace = 'robot1'
rospy.init_node(robot_namespace + '_registration_send_msg_test_node')
register(robot_namespace)  # TODO register for test


msg = Data_Position()
msg.source = "pioneer3at"
msg.destination = "Dumpster"
msg.command = "update map - test 1"
print msg.command
send_message(msg, 'Data_Position')

msg = Data_Position()
msg.source = "pioneer3at"
msg.destination = "Dumpster"
msg.command = "update map - test 2"
print msg.command

send_message(msg, 'Data_Position')

msg = Data_Position()
msg.source = "pioneer3at"
msg.destination = "Dumpster"
msg.command = "update map - test 3"
print msg.command

send_message(msg, 'Data_Position')

msg = Data_Position()
msg.source = "pioneer3at"
msg.destination = "Dumpster"
msg.command = "update map - test 4"
print msg.command

send_message(msg, 'Data_Position')
