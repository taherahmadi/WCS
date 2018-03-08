#!/usr/bin/env python
# coding=utf-8
"""Environment Information.

# Authors:  MohammadHossein GohariNejad <hoseingohari76@gmail.com>
#           Sajjad Azami <sajjadaazami@gmail.com>
# License:  BSD 3 clause

Utils package for environment information extraction.
"""
import numpy as np

import os
import signal
import sys
from time import gmtime,strftime
import rospy
from communication_node.msg import *
from nav_msgs.msg import *
from environment_information import get_object_distance ,get_n_walls_between
from propagation_models import *
propagation_parameters={ "decay_factor":2.2,"l0":40,"threshold":93}
connection_list=[];
direct_connection=[];
debuger_mode=False;
information_logger=None;
robots_list=[];
prop_model="mwm";





def line_of_sight():
    global prop_model;
    global connection_list;
    global robots_list;
    global direct_connection;
    for i in range(0,len(connection_list)):
        for j in range(0,len(robots_list)):
            if (connection_list[i][0]==robots_list[j]):continue;
            if prop_model=="1sm":
                distance = get_object_distance(robots_list[i],robots_list[j]);
                if(distance==-1 or distance==None):
                    connection_list[i][1+j]=0;
                    direct_connection[i][1+j]=0;
                    continue;
                result = one_slope_model_checker(distance=distance,decay_factor=propagation_parameters["decay_factor"],l0=propagation_parameters["l0"],threshold=propagation_parameters["threshold"])
                if(result[0]==True):
                    connection_list[i][1+j]=1;
                    direct_connection[i][1+j]=1;
                else:
                     connection_list[i][1+j]=0;
                     direct_connection[i][1+j]=0;
            elif prop_model=="mwm":
                distance_and_walls = get_n_walls_between(robots_list[i],robots_list[j]);
                if(distance_and_walls==-1 or distance_and_walls==None):
                    connection_list[i][1+j]=0;
                    direct_connection[i][1+j]=0;
                    print("problem")
                    continue;
                result = multi_wall_model_checker(distance=distance_and_walls[0],number_of_walls=distance_and_walls[1],decay_factor=propagation_parameters["decay_factor"],l0=propagation_parameters["l0"],threshold=propagation_parameters["threshold"])
                print("signal",result[1])
                if(result[0]==True):
                    connection_list[i][1+j]=1;
                    direct_connection[i][1+j]=1;
                else:
                    connection_list[i][1+j]=0;
                    direct_connection[i][1+j]=0;

def multihub():
    global connection_list;
    global robots_list;
    for i in range (0,len(connection_list)):
        for j in range (0,len(robots_list)):
            if (connection_list[i][0]==robots_list[j]):continue;
            if (connection_list[i][1+j]==1 or connection_list[j][i+1]==1):
                for k in range(0,len(connection_list)):
                    if (connection_list[k][i+1]==1 or connection_list[i][k+1]==1):
                        connection_list[k][j+1]=1;
                        connection_list[j][k+1]=1;
def main():
    global connection_list;
    global robots_list;
    global direct_connection;
    global debuger_mode;
    global information_logger;
    global prop_model;
    rospy.init_node("update_info", anonymous=True)
    print(np.log10(100),"htis is the np we are looking for")
    robots_list=rospy.get_param("/robots_list")
    for i in robots_list:
       temp_list=[i];
       for j in robots_list:
           temp_list.append(0);
       connection_list.append(list(temp_list));
       direct_connection.append(list(temp_list));
    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():
        line_of_sight();
        multihub();
        for i in range(0,len(connection_list)):
            rospy.set_param("/connection_list_"+connection_list[i][0],connection_list[i]);
        #print("update done");
    rospy.spin();

main();
