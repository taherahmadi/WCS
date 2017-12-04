#!/usr/bin/env python

""" API

# Authors:  MohammadHossein GohariNejad <mhgoharinejad@gmail.com>
# License:  BSD 3 clause

"""
import rospy
from gazebo_information_plugins.srv import *


class GetInfo:
    def __init__(self):
        self.client = None
        rospy.wait_for_service("distance_service")
        try:
            self.client = rospy.ServiceProxy("distance_service", distance_serivce)
            print ("server found")
        except rospy.ServiceException:
            print ("Service call failed ")

    def request(self, command="walls", robot1="sos1", robot2="sos2"):
        request = distance_serivceRequest(command, robot1, robot2)
        output = [None, None]
        try:
            response = self.client(request)
            output = [response.distance, response.number_of_objects]
        except rospy.ServiceException:
            print ("sending the request failed")
            response = "failed"
        return output
