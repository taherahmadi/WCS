#include <ros/ros.h>
#include <string>

template <class messageType,class reference_class>
template <class sending_message>
class messanger_api
{
protected:

  ros::NodeHandle* nh;


public:



  messanger_api(ros::NodeHandle* nodehandle) :
    nh(nodehandle),
  {
  }

  ~messanger_api(void)
  {
  }

  ros::Subscriber receive_message(std::string name_space,std::string message_tag, void(reference_class::*fp)(messageType), reference_class *obj){

    return nodehandle->subscribe(name_space+message_tag,100,fp,obj);
  }
  ros::Subscriber receive_message(std::string name_space,std::string message_tag, void(*fp)(messageType)){

    return nodehandle->subscribe(name_space+message_tag,100,fp);
  }
  void send_message(std::string message_tag,Sending_message msg){

    ros::Publisher message_pub = n.advertise<Sending_message>("/message_server_"+message_tag, 10);
    message_pub.publish(msg);
    ros::spinOnce();
  }


};
