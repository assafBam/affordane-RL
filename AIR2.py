#!/usr/bin/env python

import rospy
import yaml
import os
import argparse
import numpy as np

from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseWithCovarianceStamped
import dynamic_reconfigure.client

from geometry_msgs.msg import PoseStamped
import nav_msgs.srv

# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from MRS_236609.srv import ActionReq

from geometry_msgs.msg import Twist
from rospy import Duration

from gazebo_msgs.msg import ModelStates


SPHERE_INIT_POS_X = 0
SPHERE_INIT_POS_Y = 0
Processing_flag = False


def move_line():
    twist = Twist()
    twist.angular.x = 0  # 0.5
    twist.angular.y = 0  # 0.5
    twist.angular.z = 0  # 0.5
    twist.linear.x = 0.5
    twist.linear.y = 0
    twist.linear.z = 0

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(2)
    start_time = rospy.Time.now()
    duration = rospy.Duration.from_sec(5)
    while rospy.Time.now() - start_time < duration:
        pub.publish(twist)
        r.sleep()

    print("send stop message to the robot")
    stop_message = Twist()
    stop_message.angular.x = 0  # 0.5
    stop_message.angular.y = 0  # 0.5
    stop_message.angular.z = 0  # 0.5
    stop_message.linear.x = 0
    stop_message.linear.y = 0
    stop_message.linear.z = 0
    pub.publish(stop_message)

###########################################################################################################


class TurtleBot:
    def __init__(self):
       # Subscribe to the model_states topic and specify the callback function
        self.model_states_sub = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, callback=self.model_states_callback)
        self.initial_position = None
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped,
                         callback=self.set_initial_position)
        self.do_action = rospy.ServiceProxy('/do_action', ActionReq)
        self.time = None

        print("Waiting for an initial position...")
        while self.initial_position is None:
            continue
        print("The initial position is {}".format(self.initial_position))

    def set_initial_position(self, msg):
        initial_pose = msg.pose.pose
        self.initial_position = np.array(
            [initial_pose.position.x, initial_pose.position.y])
        print("initial position values {} , {} ".format(
            initial_pose.position.x, initial_pose.position.y))

    def model_states_callback(self, msg):
        # Get location only at the end
        if not Processing_flag:
            return
        # Extract the position of the desired object
        model_index = msg.name.index('unit_sphere')
        position = msg.pose[model_index].position

        # Access the x, y, and z coordinates
        x = position.x
        y = position.y
        z = position.z

        # Process the object's location data
        # (e.g., print it or use it in further computations)
        print("Sphere location - x:", x, "y:", y, "z:", z)

        # Unsubscribe from the model_states topic so we will get the return value only once
        self.model_states_sub.unregister()
        # Stop the ROS node and exit the program
        rospy.signal_shutdown('Sphere location processed')

    def _move_robot_to_point(self, x, y, w):
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        print("Start - wait for server")
        client.wait_for_server()
        print("End - wait for server")

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Move 0.5 meters forward along the x axis of the "map" coordinate frame
        goal.target_pose.pose.position.x = x
        # Move to position 0.5 on the y axis of the "map" coordinate frame
        goal.target_pose.pose.position.y = y
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.w = w

        # Sends the goal to the action server.
        print('Sending goal...')
        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return client.get_result()

    def move_robot_to_point(self, x, y, w=1):
        try:
            print('starting move_base')
            result = self._move_robot_to_point(x, y, w)
            if result:
                rospy.loginfo("Goal execution done!")
            else:
                rospy.loginfo("Goal unachievable!")
            return result
        except rospy.ROSInterruptException:
            rospy.loginfo("Navigation test finished.")

    def run(self, x, y):
        print("need to come closer to point {},{}".format(x, y))
        self.move_robot_to_point(x-0.5, y)
        print("start moving towards the sphere".format(x-0.5, y))
        move_line()
        print("robot finished it's job")


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    rospy.init_node('air2')

    tb3 = TurtleBot()
    print("Sphere initial position: {}, {}".format(
        SPHERE_INIT_POS_X, SPHERE_INIT_POS_Y))
    tb3.run(SPHERE_INIT_POS_X, SPHERE_INIT_POS_Y)
    # TODO: collect sphere location and print it
    Processing_flag = True  # now we will get the loction
    rospy.spin()
