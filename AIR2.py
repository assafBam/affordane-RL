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

CUBE_EDGE = 0.5
ACT_TIME = 0.1  # action time


#####################################################################
####################### legacy code from assigment 2 ################
# def get_distance_of_two_points(p1_x, p1_y, p2_x, p2_y):
#     def get_PoseStamped(x, y):
#         pose = PoseStamped()
#         pose.header.seq = 0
#         pose.header.frame_id = "map"
#         pose.header.stamp = rospy.Time(0)
#         pose.pose.position.x = float(x)
#         pose.pose.position.y = float(y)
#         return pose

#     def euclid_dist(p1, p2):
#         return (p1.pose.position.x-p2.pose.position.x)**2 + (p1.pose.position.y-p2.pose.position.y)**2
#     start = get_PoseStamped(p1_x, p1_y)
#     Goal = get_PoseStamped(p2_x, p2_y)

#     #print('Getting a plan from ({}, {}) to pose ({},{})...'.format(p1_x, p1_y, p2_x, p2_y))
#     get_plan = rospy.ServiceProxy('move_base/make_plan', nav_msgs.srv.GetPlan)
#     req = nav_msgs.srv.GetPlan()
#     req.start = start
#     req.goal = Goal
#     req.tolerance = .5
#     resp = get_plan(req.start, req.goal, req.tolerance)
#     rospy.loginfo(len(resp.plan.poses))
#     # generate a list of pairs of poses that togather make the path
#     d_poses = zip(resp.plan.poses, resp.plan.poses[1:])
#     path_length = sum(euclid_dist(pose1, pose2) for pose1, pose2 in d_poses)
#     return path_length
#####################################################################


# class graphNode:
#     def __init__(self):
#         self._ws = ""
#         self._act = ""
#         self._affordance_location = []
#         self._avaliable_tasks = []
#         self._act_history = []
#         self._reward = 0
#         self._time = 0
#         self._need_pickup = False
#         self._need_place = ""
#         self._is_terminal_state = False
#         self._predessor = None


# def getSuccessors(node, ws, tasks, time):

#     def IsUsefullProgress(node):
#         task_in_progress = "->".join(node._act_history)  # act1->act2->..
#         # checks if this new act is usefull
#         return any(task.startswith(task_in_progress) for task in node._avaliable_tasks)

#     def checkReward(node):
#         task_in_progress = "->".join(node._act_history)
#         if task_in_progress in node._avaliable_tasks:
#             # remove the task from the avaliable tasks
#             node._avaliable_tasks.remove(task_in_progress)
#             node._act_history = []  # clear act history
#             return tasks[task_in_progress]  # return reward
#         return 0

#     def calcTIme(ancestor, node):
#         robot_speed = 0.22
#         if node._need_place:
#             robot_speed = 0.1
#         return ancestor._time + ACT_TIME + (get_distance_of_two_points(ancestor._affordance_location[0], ancestor._affordance_location[1], node._affordance_location[0], node._affordance_location[1])/robot_speed)

#     # LOGIC:
#     successors_list = []
#     # then create successors of each legal combination between ws_,act_
#     if(not node._need_pickup and not node._need_place):
#         # this syntax is for the one time we come from the root
#         for ws_id in (ws if not node._act.startswith('PL') else [node._ws]):
#             for activity in ws[ws_id].tasks:
#                 if activity.startswith('ACT'):
#                     succssor = graphNode()
#                     succssor._ws = ws_id
#                     succssor._act = activity
#                     succssor._avaliable_tasks = node._avaliable_tasks  # get from ancestor
#                     # location of ws
#                     succssor._affordance_location = ws[succssor._ws].affordance_center
#                     succssor._act_history = node._act_history + \
#                         [activity]  # get from ancestor + add this activity
#                     succssor._reward = checkReward(succssor)
#                     succssor._time = calcTIme(node, succssor)
#                     # if out of time it will be None
#                     succssor._time = None if succssor._time > time else succssor._time
#                     succssor._need_pickup = True
#                     succssor._need_place = ""
#                     succssor._is_terminal_state = not IsUsefullProgress(
#                         succssor)

#                     if succssor._reward >0: #if we finished a task, than we act like a root again
#                         succssor._need_pickup = False
#                     successors_list.append(succssor)
#     elif(node._need_pickup):
#         for activity in ws[node._ws].tasks:
#             if activity.startswith('PU'):
#                 succssor = graphNode()
#                 succssor._ws = node._ws
#                 succssor._act = activity
#                 succssor._avaliable_tasks = node._avaliable_tasks  # get from ancestor
#                 # location of ws
#                 succssor._affordance_location = ws[succssor._ws].affordance_center
#                 succssor._act_history = node._act_history  # get from ancestor
#                 succssor._reward = 0
#                 succssor._time = node._time + ACT_TIME
#                 succssor._need_pickup = False
#                 succssor._need_place = activity.split(
#                     '-')[-1]  # from PU-A gets "A"
#                 succssor._is_terminal_state = False
#                 successors_list.append(succssor)
#     elif(node._need_place):
#         for ws_id in ws:
#             if ws_id == node._ws:
#                 continue  # need to go to different ws
#             for activity in ws[ws_id].tasks:
#                 # finds PL-A for example
#                 if activity.startswith('PL-'+node._need_place):
#                     succssor = graphNode()
#                     succssor._ws = ws_id
#                     succssor._act = activity
#                     succssor._avaliable_tasks = node._avaliable_tasks  # get from ancestor
#                     # location of ws
#                     succssor._affordance_location = ws[succssor._ws].affordance_center
#                     succssor._act_history = node._act_history  # get from ancestor
#                     succssor._reward = 0
#                     succssor._time = calcTIme(node, succssor)
#                     # if out of time it will be None
#                     succssor._time = None if succssor._time > time else succssor._time
#                     succssor._need_pickup = False
#                     succssor._need_place = ""
#                     succssor._is_terminal_state = False
#                     successors_list.append(succssor)

#     for successor in successors_list:
#         successor._predessor = node
#     return successors_list


# def get_path(terminal_state): #type is graphNode
#     path = []
#     node = terminal_state
#     while node._predessor:
#         path.append((node._ws, node._act))
#         node = node._predessor
#     path.reverse()
#     return path


# def BES(init_pos, ws, tasks, time):
#     '''
#         Bamberger Ephraim Search
#     '''
#     root = graphNode()
#     root._avaliable_tasks = tasks.keys()
#     root._affordance_location = init_pos
#     open_list = [root]
#     terminal_states = []
#     while open_list:
#         # gets the node with the maximal reward
#         next_node = max(open_list, key=lambda node: node._reward)
#         open_list.remove(next_node)
#         if next_node._is_terminal_state:
#             terminal_states.append(next_node)
#             continue

#         successors = getSuccessors(next_node, ws, tasks, time)

#         if all(successor._time is None for successor in successors):
#             terminal_states.append(next_node)
#             continue

#         for successor in successors:
#             if successor._time is None:
#                 continue
#             successor._reward += next_node._reward
#             open_list.append(successor)
#     return get_path(max(terminal_states, key=lambda node: node._reward))

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
        # self.time = time
        # best_path = BES(self.initial_position, ws, tasks, self.time)
        # print(best_path)
        # previous_ws =""
        # for goal_ws, goal_act in best_path:
        #     goal_ws_location = ws[goal_ws].affordance_center #(x1,y1)
        #     print("go to {} at: {} to do {}".format(goal_ws, goal_ws_location, goal_act))
        #     if previous_ws != goal_ws:
        # go to ws
        print("move to point {},{}".format(x, y))
        self.move_robot_to_point(x-0.25, y)
        print("start moving towards the sphere")
        move_line()
        print("robot finished it's job")
        #         continue
        # #do action
        # print("do action {} at: {}".format(goal_act, goal_ws))
        # res = self.do_action(goal_ws,goal_act)
        # print(res)
        # previous_ws = goal_ws


# ======================================================================================================================

# def analyse_res(msg):
#     result = {}
#     for line in msg.split("\n"):
#         if line:
#             parts = line.split(" ")
#             key = parts[0]
#             x = float(parts[-2])
#             y = float(parts[-1])
#             result[key] = [x, y]
#     return result


# class Workstation:
#     def __init__(self, location, tasks):
#         self.location = location
#         self.tasks = tasks
#         self.affordance_center = None

#     def update_affordance_center(self, new_center):
#         self.affordance_center = new_center

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    rospy.init_node('assignment3')

    # CLI = argparse.ArgumentParser()
    # CLI.add_argument(
    #     "--time",
    #     type=float,
    #     default=2.0,
    # )
    # args = CLI.parse_args()
    # time = args.time

    # gcm_client = dynamic_reconfigure.client.Client(
    #     "/move_base/global_costmap/inflation_layer")
    # gcm_client.update_configuration({"inflation_radius": 0.2})
    # lcm_client = dynamic_reconfigure.client.Client(
    #     "/move_base/local_costmap/inflation_layer")
    # lcm_client.update_configuration({"inflation_radius": 0.2})

    # ws_file = os.path.dirname(os.path.dirname(
    #     os.path.abspath(__file__))) + "/config/workstations_config.yaml"
    # tasks_file = os.path.dirname(os.path.dirname(
    #     os.path.abspath(__file__))) + "/config/tasks_config.yaml"
    # ws = {}
    # with open(ws_file, 'r') as f:
    #     data = yaml.load(f)
    #     num_ws = data['num_ws']
    #     for i in range(num_ws):
    #         ws['ws' + str(i)] = Workstation(data['ws' + str(i)]
    #                                         ['location'], data['ws' + str(i)]['tasks'])

    # rospy.wait_for_service('/affordance_service', timeout=5.0)
    # service_proxy = rospy.ServiceProxy('/affordance_service', Trigger)
    # res = service_proxy()
    # aff = analyse_res(res.message)
    # for key, val in ws.items():
    #     val.update_affordance_center(aff[key])

    # with open(tasks_file, 'r') as f:
    #     data = yaml.load(f)
    #     tasks = data['tasks']

    tb3 = TurtleBot()
    tb3.run(2.7, -1.7)
