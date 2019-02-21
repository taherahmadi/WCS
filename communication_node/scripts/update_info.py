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
propagation_parameters = {"decay_factor":2.2, "l0":40, "threshold":93}
wall_decays={"light_decay":6.0,"medium_decay":8.0,"heavy_decay":10.0}
max_range=30.0
connection_list = []
direct_connection = []
debuger_mode = False
information_logger = None
robots_list = []
propagation_model = "mwm"





def line_of_sight():
    global propagation_model
    global connection_list
    global robots_list
    global direct_connection
    global wall_decays
    global max_range
    for i in range(0,len(connection_list)):
        for j in range(0,len(robots_list)):
            if (connection_list[i][0]==robots_list[j]):continue
            if propagation_model=="one_slope":
                print("Debug line 4 ===>>> ",propagation_model)
                distance = get_object_distance(robots_list[i],robots_list[j])
                if(distance==-1 or distance==None):
                    connection_list[i][1+j]=0
                    direct_connection[i][1+j]=0
                    continue
                result = one_slope_model_checker(distance=distance,decay_factor=propagation_parameters["decay_factor"],l0=propagation_parameters["l0"],threshold=propagation_parameters["threshold"])
                if(result[0] == True):
                    connection_list[i][1+j]=1
                    direct_connection[i][1+j]=1
                else:
                     connection_list[i][1+j]=0
                     direct_connection[i][1+j]=0
            
            if propagation_model=="range_based":
                print("Debug line 4 ===>>> ",propagation_model)
                distance = get_object_distance(robots_list[i],robots_list[j])
                if(distance==-1 or distance==None):
                    connection_list[i][1+j]=0
                    direct_connection[i][1+j]=0
                    continue
                if(float(distance)<max_range):
                    connection_list[i][1+j]=1
                    direct_connection[i][1+j]=1
                else:
                     connection_list[i][1+j]=0
                     direct_connection[i][1+j]=0

            elif propagation_model == "multi_wall":
                print("Debug line 4 ===>>> ",propagation_model)
                distance_and_walls = get_n_walls_between(robots_list[i],robots_list[j])
                if(distance_and_walls==-1 or distance_and_walls==None):
                    connection_list[i][1+j]=0
                    direct_connection[i][1+j]=0
                    print("problem")
                    continue
                result = multi_wall_model_checker(distance=distance_and_walls[0],number_of_walls=distance_and_walls[1],decay_factor=propagation_parameters["decay_factor"],l0=propagation_parameters["l0"],threshold=propagation_parameters["threshold"],light_decay=wall_decays["light_decay"],medium_decay=wall_decays["medium_decay"],heavy_decay=wall_decays["heavy_decay"])
                print("signal",result[1])
                if(result[0]==True):
                    connection_list[i][1+j]=1
                    direct_connection[i][1+j]=1
                else:
                    connection_list[i][1+j]=0
                    direct_connection[i][1+j]=0

            elif propagation_model == "full_connection":
                print("Debug line 4 ===>>> ",propagation_model)
                connection_list[i][1+j]=1
                direct_connection[i][1+j]=1
            
            elif propagation_model == "single_wall":
                print("Debug line 4 ===>>> ",propagation_model)
                distance_and_walls = get_n_walls_between(robots_list[i],robots_list[j])
                if(distance_and_walls==-1 or distance_and_walls==None):
                    connection_list[i][1+j]=0
                    direct_connection[i][1+j]=0
                    print("problem")
                    continue
                result = single_wall_model_checker(distance=distance_and_walls[0],number_of_walls=distance_and_walls[1],decay_factor=propagation_parameters["decay_factor"],l0=propagation_parameters["l0"],threshold=propagation_parameters["threshold"],wall_decay=wall_decays["light_decay"])
                print("signal",result[1])
                if(result[0]==True):
                    connection_list[i][1+j]=1
                    direct_connection[i][1+j]=1
                else:
                    connection_list[i][1+j]=0
                    direct_connection[i][1+j]=0
                
            
                
def multihub():
    global connection_list
    global robots_list
    for i in range (0,len(connection_list)):
        for j in range (0,len(robots_list)):
            if (connection_list[i][0]==robots_list[j]):continue
            if (connection_list[i][1+j]==1 or connection_list[j][i+1]==1):
                for k in range(0,len(connection_list)):
                    if (connection_list[k][i+1]==1 or connection_list[i][k+1]==1):
                        connection_list[k][j+1]=1
                        connection_list[j][k+1]=1
def main():
    global connection_list
    global robots_list
    global direct_connection
    global debuger_mode
    global information_logger
    global propagation_model
    global propagation_parameters
    global wall_decays
    global max_range

    rospy.init_node("update_info", anonymous=True)
    robots_list=rospy.get_param("/robots_list")

    prop_model=rospy.get_param("prop_model",default="multi_wall")
    print("Debug line 1 ===>>> ",prop_model)
    model_names=["multi_wall","single_wall","range_based","one_slope","full_connection"]
    if not (prop_model in model_names):
        propagation_model="multi_wall"
        print("Debug line 2 ===>>> mwm")

    else :
        propagation_model=prop_model
        print("Debug line 3 ===>>> ",prop_model)

    try:
        decay_factor=float(rospy.get_param("decay_factor",default="2.2"))
        l0=float(rospy.get_param("l0",default="40"))
        threshold=float(rospy.get_param("threshold",default="93"))
        propagation_parameters.update({"decay_factor":decay_factor, "l0":l0, "threshold":threshold})
    except:
        propagation_parameters = {"decay_factor":2.2, "l0":40, "threshold":93}
    
    try:
        max_range=float(rospy.get_param("max_range",default="30.0"))
    except:
        max_range=30.0
    
    try:
        light_decay=float(rospy.get_param("light_decay",default="6"))
        medium_decay=float(rospy.get_param("medium_decay",default="8"))
        heavy_decay=float(rospy.get_param("heavy_decay",default="10"))
        wall_decays.update({"light_decay":light_decay, "medium_decay":medium_decay, "heavy_decay":heavy_decay})
    except:
        wall_decays = {"light_decay":6.0,"medium_decay":8.0,"heavy_decay":10.0}
    




    for i in robots_list:
       temp_list=[i]
       for j in robots_list:
           temp_list.append(0)
       connection_list.append(list(temp_list))
       direct_connection.append(list(temp_list))
    rate = rospy.Rate(0.5) # 10hz
    while not rospy.is_shutdown():
        line_of_sight()
        multihub()
        for i in range(0,len(connection_list)):
            rospy.set_param("/connection_list_"+connection_list[i][0],connection_list[i])
        for i in range(0,len(direct_connection)):
            rospy.set_param("/direct_connection_"+direct_connection[i][0],direct_connection[i])
        #print("update done")
    rospy.spin()

main()
