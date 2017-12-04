#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>


namespace gazebo {
    class ContactPlugin : public SensorPlugin {
    public:
        ros::Publisher collision_publisher;
        ros::NodeHandle n;

        ContactPlugin() : SensorPlugin() {

        }

        virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
            // Get the parent sensor.
            this->parentSensor =
                    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

            // Make sure the parent sensor is valid.
            if (!this->parentSensor) {
                gzerr << "ContactPlugin requires a ContactSensor.\n";
                return;
            }

            // Connect to the sensor update event.
            this->updateConnection = this->parentSensor->ConnectUpdated(
                    std::bind(&ContactPlugin::OnUpdate, this));

            // Make sure the parent sensor is active.
            this->parentSensor->SetActive(true);
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "contact_node", ros::init_options::NoSigintHandler);
            collision_publisher = n.advertise<std_msgs::String>("collision_topic", 10);

        }

        virtual void OnUpdate() {
            // Get all the contacts.
            msgs::Contacts contacts;
            contacts = this->parentSensor->Contacts();
            for (unsigned int i = 0; i < contacts.contact_size(); ++i) {
                std::string collision_name = contacts.contact(i).collision2();
                std::stringstream ss(collision_name);
                std::string item;
                getline(ss, item, ':');
                std_msgs::String object_name;
                object_name.data = item;
                collision_publisher.publish(object_name);
            }
        }

    private:
        sensors::ContactSensorPtr parentSensor;
        event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)
}
