#! /usr/bin/env python
from __future__ import print_function
import rospy

import actionlib
import communication_node.msg


def registration_client(name_space):
    # Creates the SimpleActionClient, passing the type of the action
    # (RegistrationAction) to the constructor.
    client = actionlib.SimpleActionClient('/registration', communication_node.msg.RegistrationAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = communication_node.msg.RegistrationGoal(robot_namespace=name_space)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A RegistrationResult


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('registration_client_py')
        result = registration_client("SOS1")
        print("Result:", ', '.join([str(n) for n in result.robots_list]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
