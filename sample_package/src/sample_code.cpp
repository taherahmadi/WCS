#include "ros/ros.h"



int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "hello_world");

  
  ros::NodeHandle n;

  

  ros::Rate loop_rate(0.5);


  while (ros::ok())
  {
    

    ROS_INFO("Hello World \n");

  

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
