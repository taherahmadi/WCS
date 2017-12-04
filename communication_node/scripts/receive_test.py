#!/usr/bin/env python

import rospy
from communication_node.scripts.messenger_api import receive_message
from communication_node.msg import Data_Position

print 'here1'
receive_message('Dumpster', 'Data_Position')
print 'here2'
