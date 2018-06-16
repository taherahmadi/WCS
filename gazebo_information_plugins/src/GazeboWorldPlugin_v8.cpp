/* world plugin

# Authors:  MohammadHossein GohariNejad <hoseingohari76@gmail.com>
# License:  BSD 3 clause


*/
#include <cmath>        // std::abs#include <gazebo/common/Plugin.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo_information_plugins/distance_serivce.h"
#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <math.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#define NUM_SRV 1 // this is the number of serivceservers we want to create

namespace gazebo {
    class WorldPluginTutorial ;
    class service_handle{
          /*  this class is a wrapper around a ros ServiceServer and its handler function
              for every server we create one object of this class;
             */
    public:
      WorldPluginTutorial* WPT; // this is a pointer to the wold plugin so we can use it's methodes
      std::string sn; //this is the name of the service we want to create
      ros::ServiceServer service1; //this is the serviceserver object we want to create
      service_handle(std::string service_name,WorldPluginTutorial* world_plugin); // this is the constructor for the class
      bool service_handler1(gazebo_information_plugins::distance_serivce::Request &req,
                           gazebo_information_plugins::distance_serivce::Response &res);// this function is the handler for the requests received by the serviceserver
     };
    class WorldPluginTutorial : public WorldPlugin {
      //this class is the plugin that we create it has to inheres from WorldPlugin class
      // and then we have to overload the load function inhered from WorldPlugin class
    public:
        physics::WorldPtr _world; // this is a pointer to the world object used by gazebo engine
        ros::NodeHandle n;  // a ros nodehandle
        ros::Subscriber map_subscriber;
      //  ros::ServiceServer service1,service2,service3,service4;
        std::vector<std::string> vec; // a vector to keep the unique objects when counting number of walls between two objects
        std::vector<service_handle*> vec_srv; // a vector to keep pointer to  the objects of type service_handle
        std::string object_name;
        int check; // a flag variable indicating that we want to save the data received from contact sensor

        WorldPluginTutorial() : WorldPlugin() {}


        std::vector<int> get_walls(std::string robot_name1, std::string robot_name2) {
            // this function returns number of walls between robot_name1 and robot_name2 as Integer
            //if the robot_name1 or robot_name2 are not name for models in the world this function returns -1
            std::vector<int> type_vec;
            physics::ModelPtr model1, model2;
            model1 = _world->GetModel(robot_name1);
            model2 = _world->GetModel(robot_name2);
            if (model1 == 0 || model2 == 0) {
                ROS_WARN("invalid model name!");
                type_vec.push_back(-1);
                return type_vec;
            }

            float start_x, start_y, slope;
            int iterations;
            int my_temp=1;
            float increment=0.01;
            float x_1=model1->GetWorldPose().pos.x;
            float y_1=model1->GetWorldPose().pos.y;
            float x_2=model2->GetWorldPose().pos.x;
            float y_2=model2->GetWorldPose().pos.y;

            float x_temp=std::abs(x_1 -x_2);
            float y_temp=std::abs(y_1 -y_2);
            if (x_temp>y_temp){
              iterations = x_temp * 100;
            }
            else {
              iterations = y_temp * 100;
              increment=x_temp/iterations;
            }

            if (x_1 >x_2 ) {
                start_x = x_2;
                start_y = y_2;
                slope = (y_1 - start_y) / (x_1 - start_x);

              }
            else if (x_1 == x_2){
               my_temp=0;
              start_x = x_1;
              start_y = y_1;
              slope = 1;
              increment=0.01;

            } else {
                start_x = x_1;
                start_y = y_1;
                slope = (y_2 - start_y) / (x_2- start_x);
            }
          //  ROS_WARN("iterations %d \n", iterations);

            std::string test_object = "unique_sphere";
            vec.clear();
            vec.push_back(robot_name1);
            vec.push_back(robot_name2);
            check = 1;
            _world->GetModel(test_object)->SetStatic(true);

            ros::Rate r(12*iterations);

            for (int i = 0; i < iterations; i++) {
              _world->GetModel(test_object)->SetStatic(true);

                math::Pose new_test_pose(start_x + i * increment*my_temp, start_y + i * increment * slope, 0.12, 0, 0, 0);
                _world->GetModel(test_object)->SetWorldPose(new_test_pose);
              //  this->object_name = "";
                _world->GetModel(test_object)->SetWorldPose(new_test_pose);
              //  ROS_WARN("pose x: %f y: %f  \n", start_x + i * 0.02, start_y + i * 0.02 * slope);
                // for (int j = 0; j < vec.size(); j++) {
                //     if (vec[j] == this->object_name) { break; }
                //     else if (j + 1 == vec.size() && this->object_name != "") {
                //         vec.push_back(this->object_name);
                //         ROS_INFO("these are the objects %s  \n", this->object_name.c_str());
                //     }
                // }
                r.sleep();
            }

            int number_of_walls = vec.size() - 2;
            check = 0;
            std::string Hard_wall="Hard_wall",Soft_wall="Soft_wall",Medium_wall="Medium_wall";

            int hard_num=0,soft_num=0,medium_num=0;
            for (int k=0;k<vec.size();k++){
              if(vec[k].length()<9){soft_num++;}
              else if (!Soft_wall.compare(vec[k].substr(0,9))){soft_num++;}
              else if (!Hard_wall.compare(vec[k].substr(0,9))){hard_num++;}
              else if (!Medium_wall.compare(vec[k].substr(0,11))){medium_num++;}
              else {soft_num++;}
            }
            type_vec.push_back(soft_num);
            type_vec.push_back(medium_num);
            type_vec.push_back(hard_num);
            vec.clear();
            math::Pose new_test_pose(200.0, 200.0, 10.0, 0, 0, 0);
            _world->GetModel(test_object)->SetWorldPose(new_test_pose);
            return type_vec;

        }

        float get_distance(std::string robot_name1, std::string robot_name2) {
             //this function returns the distance between robot_name1 and robot_name2 as float
             // if the robot_name1 or robot_name2 are not valid models this function returns -1
            physics::ModelPtr model1, model2;
            model1 = _world->GetModel(robot_name1);
            model2 = _world->GetModel(robot_name2);
            if (model1 == 0 || model2 == 0) {

                ROS_WARN("invalid model name!");
                return -1;
            } else {
                float x_diff = model1->GetWorldPose().pos.x - model2->GetWorldPose().pos.x;
                float y_diff = model1->GetWorldPose().pos.y - model2->GetWorldPose().pos.y;

                return sqrt((x_diff * x_diff) + (y_diff * y_diff));
            }

        }

        void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
           //this function is inhered from WorldPlugin reference class and we have to override it
           // when the gazebo enging starts and this plugin loads this function is automatically RECALLED
           // _parent is a pointer to the world object used by gazebo engine
            std::string test_object = "unique_sphere";
            check = 0;
            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized()) {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }
            _world = _parent;

            ROS_INFO("Hello World!");

            //here we create a SDF object with a contact sensor attached to it and insert it into the world
            // to count the number of objects that this SDF object touches
            sdf::SDF sphereSDF;
            sphereSDF.SetFromString(
                    "<sdf version ='1.4'>\
        <model name ='test_sphere'>\
        <pose>-150 -150 0.2 0 0 0</pose>\
        <link name ='link'>\
        <sensor name='my_contact' type='contact'>\
        <plugin name='my_plugin' filename='libgazebo_plugin_contact.so'/>\
        <contact>\
        <collision>test_collision</collision>\
        </contact>\
        </sensor>\
        <pose>0 0 0 0 0 0</pose>\
        <collision name ='test_collision'>\
        <geometry>\
        <sphere><radius>0.01</radius></sphere>\
        </geometry>\
        </collision>\
        <visual name ='test_visual'>\
        <geometry>\
        <sphere><radius>0.01</radius></sphere>\
        </geometry>\
        </visual>\
        </link>\
        </model>\
        </sdf>");
            // Demonstrate using a custom model name.
            sdf::ElementPtr model = sphereSDF.Root()->GetElement("model");
            model->GetAttribute("name")->SetFromString("unique_sphere");
            _world->InsertModelSDF(sphereSDF);
            ROS_INFO("Hello World!");


            int argc = 0;
            char **argv = NULL;
            // we create a ros node to be able to set up services
            ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            for (int i=0;i<NUM_SRV;i++){
              // we create objects of type service_handle and push them into vec_src
              vec_srv.push_back(new service_handle("GzInfo_service"+(std::to_string(i)),this));
            };
        //    _world->GetModel(test_object)->SetStatic(true);

            std::string subscribing_topic = "collision_topic";
            map_subscriber = n.subscribe(subscribing_topic, 200, &WorldPluginTutorial::collision_callback, this);

        }

        void collision_callback(const std_msgs::String object_name) {
          std::string concrete="concrete",willowgarage="willowgarage",brick_wall="brick";
          if(object_name.data.length()<5){return;}
          else if (!brick_wall.compare(object_name.data.substr(0,5))){
                    ROS_INFO("new objects %s",object_name.data.c_str());
                  }
          else if (!concrete.compare(object_name.data.substr(0,8))){
            ROS_INFO("new objects %s",object_name.data.c_str());}
          else if (!willowgarage.compare(object_name.data.substr(0,12))){
            ROS_INFO("new objects %s",object_name.data.c_str());}
            else{return;}
            //object's name = (jersey)(worlds_1)(longwall)(drc_practice)(Dumpster)(room1)
            this->object_name = object_name.data;
              ROS_INFO("new objects %s",object_name.data.c_str());
              vec.push_back(object_name.data);
            // if (vec.size() > 0 && check == 1) {
            //     for (int j = 0; j < vec.size(); j++) {
            //         if (vec[j] == object_name.data) {
            //             break;
            //         } else if (j + 1 == vec.size() && object_name.data != "") {
            //             vec.push_back(object_name.data);
            //             ROS_INFO("these are the objects %s  \n", object_name.data.c_str());
            //         }
            //     }
            //
            // }
        }

    };



    service_handle::service_handle(std::string service_name,WorldPluginTutorial* world_plugin){
                // this is the constructor for the service_handle class
                // service_name is the name of the serviceserver we want to create
                // world_plugin is a pointer to the object of this plugin we have created
               sn=service_name;
               WPT=world_plugin;
                 service1 = (WPT->n).advertiseService(sn, &service_handle::service_handler1, this);
    };
    bool service_handle::service_handler1(gazebo_information_plugins::distance_serivce::Request &req,
                         gazebo_information_plugins::distance_serivce::Response &res) {
        //this is the handler function for the serviceserver we have created
        // req is the reference to the object of request we have recived from a client
        // res is the reference to the object of response that we have to complete and send back to the client
        std::string command = req.command, robot1 = req.robot1, robot2 = req.robot2;

        if (command == "distance") {
            res.distance = WPT->get_distance(robot1, robot2);
            res.number_of_objects = 0;
        } else if (command == "walls") {
            res.distance = WPT->get_distance(robot1, robot2);
            res.objects_type=WPT->get_walls(robot1, robot2);

        }
        else if (command == "temp") {
            res.distance = 0;
            res.number_of_objects = 0;
        }
        else if (command == "pressure") {
            res.distance = 0;
            res.number_of_objects = 0;
        }

        return true;
    };

    // we register the plugin class we have created with the gazebo
    GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial);
}
