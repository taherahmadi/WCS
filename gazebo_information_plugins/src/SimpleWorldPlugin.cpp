/* world plugin

# Authors:  Mohammad Hossein Gohari Nejad <mhgoharinejad@gmail.com>
# License:  BSD 3 clause


*/
#include <gazebo/common/Plugin.hh>
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


namespace gazebo {
    class WorldPluginTutorial : public WorldPlugin {
    public:
        physics::WorldPtr _world;
        ros::NodeHandle n;
        ros::Subscriber map_subscriber;
        ros::ServiceServer service;
        std::vector<std::string> vec;
        std::string object_name;
        int check;

        WorldPluginTutorial() : WorldPlugin() {}

      double get_Temperature(){return _world->Atmosphere ().Temperature(0.0);}
      double get_Pressure(){return _world->Atmosphere ().Pressure(0.0);}
    //TODO    ignition::math::Vector3d get_Magnetic_Field(){return _world->MagneticField(); }

        int get_walls(std::string robot_name1, std::string robot_name2) {

            physics::ModelPtr model1, model2;
            model1 = _world->GetModel(robot_name1);
            model2 = _world->GetModel(robot_name2);
            if (model1 == 0 || model2 == 0) {
                ROS_WARN("invalid model name!");
                return -1;
            }

            float start_x, start_y, slope;
            int iterations;
            if (model1->GetWorldPose().pos.x > model2->GetWorldPose().pos.x) {
                start_x = model2->GetWorldPose().pos.x;
                start_y = model2->GetWorldPose().pos.y;
                slope = (model1->GetWorldPose().pos.y - start_y) / (model1->GetWorldPose().pos.x - start_x);
                iterations = (model1->GetWorldPose().pos.x - start_x) * 50;
            } else {
                start_x = model1->GetWorldPose().pos.x;
                start_y = model1->GetWorldPose().pos.y;
                slope = (model2->GetWorldPose().pos.y - start_y) / (model2->GetWorldPose().pos.x - start_x);
                iterations = (model2->GetWorldPose().pos.x - start_x) * 50;
            }
            ROS_WARN("iterations %d \n", iterations);
            std::string test_object = "unique_sphere";
            vec.clear();
            vec.push_back(robot_name1);
            vec.push_back(robot_name2);
            check = 1;
            _world->GetModel(test_object)->SetStatic(true);
            for (int i = 0; i < iterations; i++) {

                math::Pose new_test_pose(start_x + i * 0.02, start_y + i * 0.02 * slope, 0.05, 0, 0, 0);
                _world->GetModel(test_object)->SetWorldPose(new_test_pose);
                this->object_name = "";
                _world->GetModel(test_object)->SetWorldPose(new_test_pose);
                ROS_WARN("pose x: %f y: %f  \n", start_x + i * 0.02, start_y + i * 0.02 * slope);
                for (int j = 0; j < vec.size(); j++) {
                    if (vec[j] == this->object_name) { break; }
                    else if (j + 1 == vec.size() && this->object_name != "") {
                        vec.push_back(this->object_name);
                        ROS_INFO("these are the objects %s  \n", this->object_name.c_str());
                    }
                }
            }
            int number_of_walls = vec.size() - 2;
            check = 0;
            vec.clear();
            math::Pose new_test_pose(0.0, 0.0, 90.0, 0, 0, 0);
            _world->GetModel(test_object)->SetWorldPose(new_test_pose);
            return number_of_walls;

        }

        float get_distance(std::string robot_name1, std::string robot_name2) {
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

            check = 0;
            // Make sure the ROS node for Gazebo has already been initialized
            if (!ros::isInitialized()) {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }
            _world = _parent;
            _world->Atmosphere();
            ROS_INFO("Hello World!");
            ROS_INFO("Hello World!");

            ROS_INFO("Hello World!");


            sdf::SDF sphereSDF;
            sphereSDF.SetFromString(
                    "<sdf version ='1.4'>\
        <model name ='test_sphere'>\
        <pose>-10 -12 0.2 0 0 0</pose>\
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

            //ros::WallDuration(15.0).sleep();

            //std::string asfj="asphalt_plane";
            //std::string hi=_world->GetModel(asfj)->GetName();
            //ROS_INFO("%s",hi.c_str());
            //ROS_INFO("%f",_world->GetModel(1)->GetWorldPose().pos.x );
            //ROS_INFO("%f",get_distance("asphalt_plane","asphalt_plane_1"));
            //math::Pose asdfjk(20,20,0,0,0,0);
            //_world->GetModel(asfj)->SetWorldPose(asdfjk);
            //boost::bind(&MoveBase::goalCB, this, _1)
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
            service = n.advertiseService("GzInfo_service", &WorldPluginTutorial::service_handler, this);
            std::string subscribing_topic = "collision_topic";//move_base/local_costmap/costmap";
            map_subscriber = n.subscribe(subscribing_topic, 10, &WorldPluginTutorial::collision_callback, this);

        }

        void collision_callback(const std_msgs::String object_name) {
            this->object_name = object_name.data;
            if (vec.size() > 0 && check == 1) {
                for (int j = 0; j < vec.size(); j++) {
                    if (vec[j] == object_name.data) {
                        break;
                    } else if (j + 1 == vec.size() && object_name.data != "") {
                        vec.push_back(object_name.data);
                        ROS_INFO("these are the objects %s  \n", object_name.data.c_str());
                    }
                }

            }
            //ROS_INFO("%s",object_name.data.c_str());
        }

        bool service_handler(gazebo_information_plugins::distance_serivce::Request &req,
                             gazebo_information_plugins::distance_serivce::Response &res) {

            std::string command = req.command, robot1 = req.robot1, robot2 = req.robot2;
            if (command == "distance") {
                res.distance = get_distance(robot1, robot2);
                res.number_of_objects = 0;
            } else if (command == "walls") {
                res.distance = get_distance(robot1, robot2);
                res.number_of_objects = get_walls(robot1, robot2);
            }
            else if (command == "temp") {
                res.distance = get_Temperature()-273;
                res.number_of_objects = 0;
            }
            else if (command == "pressure") {
                res.distance = get_Pressure();
                res.number_of_objects = 0;
            }
        /*TODO   else if (command == "magnet") {
                res.distance = get_distance(robot1, robot2);
                res.number_of_objects = get_walls(robot1, robot2);
            } */
            return true;
        }


    };

    GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}
