#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <communication_node/RegistrationAction.h>

class RegistrationAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<communication_node::RegistrationAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  communication_node::RegistrationFeedback feedback_;
  communication_node::RegistrationResult result_;
  // Construct a string vector of name of robots registered for parameter server
  std::vector<std::string> robots_list_;
  // counter of robots registered
  int robot_counter_;

public:

  RegistrationAction(std::string name) :
    as_(nh_, name, boost::bind(&RegistrationAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
    robot_counter_ = 0;
    robots_list_.clear();
    nh_.getParam("robots_list", robots_list_);
  }

  ~RegistrationAction(void)
  {
  }

  void executeCB(const communication_node::RegistrationGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    robots_list_.clear();
    nh_.getParam("robots_list", robots_list_);


    std::string temp = "registering " ;
    std::string new_robot_name = goal->robot_namespace;

    // status
    feedback_.status = temp + new_robot_name;

    // checking some conditions ...
    if(std::find(robots_list_.begin(), robots_list_.end(), new_robot_name.c_str()) == robots_list_.end()){

      // publish info to the console for the user
      ROS_INFO("%s: Executing, registering robot: %s", action_name_.c_str(), new_robot_name.c_str());

      // start executing the action
      // add robto_namespace to the robots_list and increase robot_counter by one
      robots_list_.push_back(goal->robot_namespace);
      robot_counter_++;

      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
          success = false;
        as_.setPreempted();

        // break;
      }

      // add registered robot to the parameter robots_list

      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
    //  r.sleep();

      if(success)
      {
          nh_.setParam("robots_list", robots_list_);
        result_.robots_list = robots_list_;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
      }
    }else{
      result_.robots_list = robots_list_;
      ROS_ERROR("cannot register. namespace %s is registered before!",new_robot_name.c_str());
      as_.setAborted(result_);
    }

  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "registration");
  ROS_INFO("registration server started");
  RegistrationAction registration("registration");
  ros::spin();

  return 0;
}
