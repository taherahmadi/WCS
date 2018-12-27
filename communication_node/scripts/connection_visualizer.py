#!/usr/bin/env python
# coding=utf-8
"""Environment Information.

# Authors:  MohammadHossein GohariNejad <hoseingohari76@gmail.com>
#           Sajjad Azami <sajjadaazami@gmail.com>
# License:  BSD 3 clause

Utils package for environment information extraction.
"""
import rospy
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point , PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry



class Robot_Pose():
    def __init__(self,robot_namespace):
        self.robot_x=0
        self.robot_y=0
        self.robot_namespace=robot_namespace
        self.odom_subscriber=rospy.Subscriber("/"+robot_namespace+"/odom", Odometry, self.odom_callback)

    def odom_callback(self,odom_data):
        self.robot_x=odom_data.pose.pose.position.x
        self.robot_y=odom_data.pose.pose.position.y


    def get_pose(self):
        return [self.robot_x,self.robot_y]


class Visualizer():
    def __init__(self):
        self.connection_visualization_publisher=rospy.Publisher("/connection_graph", MarkerArray, queue_size=100)
        self.robots_list=rospy.get_param("/robots_list")
        self.robots_pose={}
        self.robots_index={}
        self.marker_id=0
        for i in range(len(self.robots_list)):
            self.robots_pose.update({self.robots_list[i]:Robot_Pose(robot_namespace=self.robots_list[i])})
            self.robots_index.update({self.robots_list[i]:i})
        
    def start_visualization(self):
        rate= rospy.Rate(0.2)
        while(not rospy.is_shutdown()):
            connection_graph=[]
            for i in range (len(self.robots_list)):
                for j in range (i+1,len(self.robots_list)):
                    connection_graph.append(self.create_marker(start=self.robots_list[i],end=self.robots_list[j]))
            self.connection_visualization_publisher.publish(connection_graph)
            rate.sleep()
    
    def create_marker(self,start,end):
        self.marker_id+=1
        start_pose=self.robots_pose[start].get_pose()
        end_pose=self.robots_pose[end].get_pose()
        connection_list=rospy.get_param("/direct_connection_"+start)
        connection_status=connection_list[1+self.robots_index[end]]
        connection_marker=Marker()
        connection_marker.header.frame_id = "/map"
        connection_marker.header.stamp = rospy.Time.now()
        connection_marker.ns = "points_and_lines"
        connection_marker.id = self.marker_id
        connection_marker.type = Marker.LINE_STRIP
        connection_marker.action = Marker.ADD
        connection_marker.pose.orientation.x = 0.0
        connection_marker.pose.orientation.y = 0.0
        connection_marker.pose.orientation.z = 0.0
        connection_marker.pose.orientation.w = 1.0
        connection_marker.scale.x = 0.1
        connection_marker.color.r = 1-int(connection_status)
        connection_marker.color.g = int(connection_status)
        connection_marker.color.b = 0.2
        connection_marker.color.a = 1.0
        connection_marker.points.append(Point(start_pose[0],start_pose[1],0.0))
        connection_marker.points.append(Point(end_pose[0],end_pose[1],0.0))
        connection_marker.lifetime = rospy.Duration(4.0)
        return connection_marker



if __name__ == '__main__':
    rospy.init_node("connection_visualizer", anonymous=True)
    visualization=Visualizer()
    visualization.start_visualization()


    