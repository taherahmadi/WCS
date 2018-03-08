#!/usr/bin/env python
# coding=utf-8
"""Environment Information.

# Authors:  MohammadHossein GohariNejad <hoseingohari76@gmail.com>
#           Sajjad Azami <sajjadaazami@gmail.com>
# License:  BSD 3 clause

Utils package for environment information extraction.
"""

import tf
import rospy
import rosservice
from gazebo_information_plugins.srv import *


class GetInfo:
    """ this class is a wrapper around a client for rosservice and a call back
    function for the responses received from the service


    """
    def __init__(self,service_name=""):
        """  this is the constructor for the GetInfo class
        :parameter
        service_name : string, allowed values = name of the service with the type distance_serivce
        ---------
        """
        rospy.wait_for_service(service_name)
        try:
                self.client=rospy.ServiceProxy(service_name, distance_serivce)
                print ("gazebo ",service_name," found")
        except rospy.ServiceException:
                print ("connecting to gazebo ", service_name , " failed")

    def request_func(self, command="walls", robot1="robot1", robot2="robot2"):
        """sends a request to the gazebo server for information
        :parameter
        command : string, allowed values = walls  distance
        robot1,robot2: string, allowed values = name of any model peresent in simulation
        -------
        :returns
        a list with 2 elemnts "distance" attribute of the response object and
        number_of_objects attribute of response object

        ---------
        """
        request = distance_serivceRequest(command, robot1, robot2)
        output = [None, None,None]
        try:
            response = self.client(request)
            output = [response.distance, response.number_of_objects, response.objects_type]
        except Exception as e:
            print (e)
            #print ("sending the request failed")
            response = "failed"
            #print(command,"---",robot1,"---",robot2)
        return output


server_list=rosservice.rosservice_find("gazebo_information_plugins/distance_serivce") #  we call rosservice_find to find any service with the distance_srvice type
environment_info=[]
for i in server_list:
    environment_info.append(GetInfo(i))
counter=0
def get_current_position(name_space):
    """Get current location of robot using tf translation
    :parameter
    namespace : string, robot namespace

    :returns
    location : float, location of robot

    Relations
    ----------
    subscribes from /map, /base_link topics,
    """
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    flag = True
    transform = 0
    while flag and not rospy.is_shutdown():
        try:
            # TODO lacks compatibility, uses /map which may be not available
            (transform, rot) = listener.lookupTransform((name_space + '/map'), (name_space + '/base_link'),
                                                        rospy.Time(0))
            flag = False
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    return transform


def get_n_walls_between(ns1, ns2):
    """Returns number of objects between object1 and object2 in Gazebo
    as well as distance between them
    :parameter
    ns1 : string, object1 namespace

    ns2 : string, object2 namespace

    :returns
    number_of_walls  : integer, number of all the objects between object1 and object2
    should the name of object1 or object2 be wrong or no model can be found with given names
    this method returns -1
    Relations
    ----------
    """
    global counter
    global environment_info
    global server_list
    if counter >=len(environment_info) :
        counter =0
    output_info = (environment_info[counter]).request_func(command="walls", robot1=ns1, robot2=ns2)
    counter=counter+1
    distance = output_info[0]
    number_of_walls = output_info[2]
    if distance == -1:
        print("wrong model name")
        return -1
    return [distance,number_of_walls]


def get_object_distance(ns1, ns2):
    """Returns  the distance between object1 and object2  in Gazebo
    :parameter
    ns1 : string, object1 namespace

    ns2 : string, object2 namespace

    :returns
    distance : float, distance between 2 robot

    Relations
    ----------
    """
    global counter
    global environment_info
    global server_list
    if counter >=len(environment_info) :
        counter =0
    output_info = (environment_info[counter]).request_func(command="distance", robot1=ns1, robot2=ns2)
    counter=counter+1
    distance = output_info[0]
    if distance == -1:
        print("wrong model name")
    return distance
