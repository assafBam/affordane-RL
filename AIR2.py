#!/usr/bin/env python
from __future__ import print_function
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
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os

###########################################################
########### log the position of the ball helper ###########
PIPE_ENV_VAR = 'AIR2_PIPE_WRITE'
PIPE_R_ENV_VAR = 'AIR2_PIPE_READ'
pipe_write_fd = int(os.environ.get(PIPE_ENV_VAR, os.sys.stdout.fileno()))
pipe_read_fd = int(os.environ.get(PIPE_R_ENV_VAR, os.sys.stdout.fileno()))

def final_position_logger(x,y,z):
    os.write(pipe_write_fd, bytes('({X},{Y},{Z})\n'.format(X=x,Y=y,Z=z)))
def get_V_theta():
    print('reaf pipe fd: {}'.format(pipe_read_fd))
    pipe_read_file = os.fdopen(pipe_read_fd)
    return eval(pipe_read_file.read())


SPHERE_INIT_POS_X = 0
SPHERE_INIT_POS_Y = 0
Processing_flag = False
SPHERE_END_POS_X = None
SPHERE_END_POS_Y = None
SPHERE_END_POS_Z = None
ROBOT_DISTANCE_FROM_BALL = 1

def move_line(vel):
    twist = Twist()
    twist.angular.x = 0  # 0.5
    twist.angular.y = 0  # 0.5
    twist.angular.z = 0  # 0.5
    twist.linear.x = vel  # 0.22 is the max in turtulebot3 burger
    twist.linear.y = 0
    twist.linear.z = 0

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(10)
    start_time = rospy.Time.now()
    # TODO: set shorter interval, check location of sphere and save it, and if the location didn't change between 2 intervals, stop.
    # TODO: get the location of the sphere by setting Proccesing_flag to True
    duration = rospy.Duration.from_sec((ROBOT_DISTANCE_FROM_BALL*1/vel)+0.3)#time to stop according to X=VT
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
        global SPHERE_END_POS_X
        global SPHERE_END_POS_Y
        global SPHERE_END_POS_Z
        # Get location only at the end
        if not Processing_flag:
            return
        # Extract the position of the desired object
        model_index = msg.name.index('unit_sphere')
        position = msg.pose[model_index].position

        # Access the x, y, and z coordinates
        SPHERE_END_POS_X = position.x
        SPHERE_END_POS_Y = position.y
        SPHERE_END_POS_Z = position.z

        # Process the object's location data
        # (e.g., print it or use it in further computations)
        # print("Sphere location - x:", SPHERE_END_POS_X, "y:", SPHERE_END_POS_Y, "z:", SPHERE_END_POS_Z)

        # Unsubscribe from the model_states topic so we will get the return value only once
        # self.model_states_sub.unregister()
        return
       

    def _move_robot_to_point(self, Theta_based_pose=PoseStamped()):
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # Waits until the action server has started up and started listening for goals.
        client.wait_for_server()

        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        # goal.target_pose = Theta_based_pose
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = Theta_based_pose.pose.position.x
        goal.target_pose.pose.position.y = Theta_based_pose.pose.position.y
        goal.target_pose.pose.orientation.x = Theta_based_pose.pose.orientation.x
        goal.target_pose.pose.orientation.y = Theta_based_pose.pose.orientation.y
        goal.target_pose.pose.orientation.z = Theta_based_pose.pose.orientation.z
        goal.target_pose.pose.orientation.w = Theta_based_pose.pose.orientation.w

        # Sends the goal to the action server.
        print('Sending goal using move base to {},{},with orientation{}'.format(
            Theta_based_pose.pose.position.x, Theta_based_pose.pose.position.y, Theta_based_pose.pose.orientation.w))
        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            # Result of executing the action
            return client.get_result()

    def move_robot_to_point(self, Theta_based_pose):
        try:
            print('starting move_base')
            result = self._move_robot_to_point(Theta_based_pose)
            if result:
                print("Goal execution done!")
            else:
                print("Goal unachievable!")
            return result
        except rospy.ROSInterruptException:
            print("Navigation test finished.")

    def run(self, V, Theta):
        # TODO: get V and theta, and understand how to locate the robot in the position that represant the Theta
        Theta_based_pose = PoseStamped()
        Theta_based_pose = calculate_position(Theta)  # calculate the wanted position
        print("based of Theat {},robot need to move to position {},{},{} and orientation {},{},{},{}".format(Theta, Theta_based_pose.pose.position.x, Theta_based_pose.pose.position.y,
              Theta_based_pose.pose.position.z, Theta_based_pose.pose.orientation.x, Theta_based_pose.pose.orientation.y, Theta_based_pose.pose.orientation.z, Theta_based_pose.pose.orientation.w))
        self.move_robot_to_point(Theta_based_pose)
        print("start moving towards the sphere in {}, {}".format(
            SPHERE_INIT_POS_X, SPHERE_INIT_POS_Y))
        # TODO: move the robot using move line but pass as argument the V. make sure that the V is legal.
        move_line(V)
        print("robot finished it's job and ball is rolling")


def calculate_position(theta_rad):
    # Convert theta to radians
    #theta_rad = math.radians(theta_deg)
    # Calculate the new position
    desired_distance = ROBOT_DISTANCE_FROM_BALL
    new_x = desired_distance * math.cos(theta_rad)
    new_y = desired_distance * math.sin(theta_rad)
    # Create a PoseStamped message
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = new_x
    goal_pose.pose.position.y = new_y
    # Calculate the orientation to face towards the origin (0,0)
    # Calculate the angle towards the origin
    orientation_rad = math.atan2(-new_y, -new_x)
    orientation_quaternion = quaternion_from_euler(0, 0, orientation_rad)
    goal_pose.pose.orientation.x = orientation_quaternion[0]
    goal_pose.pose.orientation.y = orientation_quaternion[1]
    goal_pose.pose.orientation.z = orientation_quaternion[2]
    goal_pose.pose.orientation.w = orientation_quaternion[3]
    return goal_pose

#wrapper function!
#input: V in m/s and Theta is radians
#output: location of the sphere after 5 seconds 
def robot_move_and_hit(V, Theta):
    global Processing_flag
    rospy.init_node('air2')

    tb3 = TurtleBot()
    print("Sphere initial position: {}, {}".format(
        SPHERE_INIT_POS_X, SPHERE_INIT_POS_Y))

    tb3.run(V, Theta)
    Processing_flag = True  # now we will get the loction
    rospy.sleep(3) #wait for a ball to roll for 3 seconds and then get it location
    print("cordinates at the end are {},{},{}".format(SPHERE_END_POS_X,SPHERE_END_POS_Y,SPHERE_END_POS_Z))
        
    # Stop the ROS node and exit the program
    # rospy.signal_shutdown('Sphere location processed')
    # rospy.spin()
    return SPHERE_END_POS_X,SPHERE_END_POS_Y, SPHERE_END_POS_Z

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    print('starting')
    v,theta = get_V_theta()
    print('v:{}, theta:{}'.format(v,theta))
    x,y,z = robot_move_and_hit(v, theta) #0.22 M/S AND 180
    print("ball's final location is {},{},{}".format(x,y,z))
    final_position_logger(x,y,z)