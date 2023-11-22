#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose

def spawn_model():
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        model_state = ModelState()
        model_state.model_name = 'uav'
        model_state.pose = Pose()  # Set the desired pose here
        model_state.twist.linear.x = -25.0
        model_state.twist.linear.y = -10.0
        model_state.twist.linear.z = 0.0
        model_state.twist.angular.x = 0.0
        model_state.twist.angular.y = 0.0
        model_state.twist.angular.z = 0.0

        response = set_model_state(model_state)
        print(response)

    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('spawn_model_node')
    spawn_model()
