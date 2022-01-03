#!/usr/bin/env python
import random
import rospy
import math
import numpy as np
import os
import sys
import ast
import pickle
import datetime
from nav_msgs.msg import Odometry
from collections import deque

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler


global graphFile, x_off, y_off, rotation_angle, abspath, time_step, time_step_limit, simulation_time_step_limit
graphFile = '/home/rachel/SearchandRescue/environment/School_Gazebo_VGraph.txt'
#graphFile = '/home/samhong/Desktop/Research/SearchandRescue/Scripts/School_Gazebo_VGraph.txt'
x_off, y_off = -34.5, -23
rotation_angle = 0
abspath = os.path.dirname(os.path.abspath(__file__))
time_step = -1
time_step_limit = 60
simulation_time_step_limit = 100
inputBelief = [ [2, 6, 7, 40, 34, 8, 10, 14, 20, 26, 32, 46, 41], [2, 4, 11, 40, 37, 12, 10, 18, 24, 30, 36, 41, 43], \
               [5, 4, 9, 16, 22, 10, 8, 18, 24, 30, 36, 46, 44], [2, 6, 7, 39, 34, 8, 10, 14, 20, 26, 32, 41, 42], \
               [4, 6, 7, 31, 19, 8, 10, 14, 20, 26, 32, 43, 46], [6, 5, 11, 31, 38, 12, 8, 18, 24, 30, 36, 46, 41], \
                [6, 5, 7, 28, 40, 8, 10, 14, 20, 26, 32, 46, 45], [3, 5, 9, 28, 16, 10, 8, 15, 21, 27, 33, 42, 44], \
                [5, 6, 11, 16, 40, 12, 8, 14, 20, 26, 32, 44, 43], [2, 6, 7, 28, 22, 8, 10, 14, 20, 26, 32, 46, 42] ]
inputTrue = [7, 10, 22, 41, 26, 18, 14, 33, 20, 14]

# --- helper functions to extract and transform points from the txt file --- #
def changeCoordinates(points):
    x, y = points
    theta = np.radians(rotation_angle)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s, x_off), (s, c, y_off), (0, 0, 1)))

    temp = np.array((x, y, 1))
    output = np.dot(R, temp)
    return [round(i, 3) for i in output]

def getCoordinates(node, lookUp):
    if node != -1:
        row = lookUp[node]
        transformed_point = changeCoordinates(np.array((row[0], row[1])))
        return [transformed_point[0], transformed_point[1], 0]
    return [-1, -1, -1]

def getAngles(node, lookUp):
    # row = lookUp[node - 1]
    # return '{}'.format(row[4])
    return [0]

def getPathCoordinates(path, lookUp):
    temp = []
    for i in path:
        temp += getCoordinates(i, lookUp)
    return temp

def getPathAngles(path, lookUp):
    temp = []
    for i in path:
        temp += getAngles(i, lookUp)
    return temp

"""
Victim modeling
- Creates a belief vector b of length n+1
- Returns belief vector b and the true position of the target
"""
def victimLocation(n = 46, list_v = None, true_v = None):
    belief_v = [0.0] * (int(n)+1)
    if not(list_v):
        list_v = [0]*5
        if not(true_v) or true_v <= 0:
            list_v = random.sample(range(1, n+1), 5)
            true_v = list_v[0]
        else:
            list_v[0] = true_v
            l = random.sample(range(1, n + 1), 4)
            while true_v in l:
                l = random.sample(range(1, n + 1), 4)
            for i in range(len(l)):
                list_v[i+1] = l[i]
    else:
        if not(true_v) or true_v <= 0:
            true_v = list_v[-3]
    contains = False
    for i in list_v:
        if i == true_v: contains = True
        belief_v[i] = 1./len(list_v)
    if not contains:
        print('true_v not in belief vector')
        sys.exit()
    return belief_v, true_v

"""
Run MILP planner from command line
Call algorithm with python3
"""
def call_planner():
    file_name = abspath + '/planner/call_my_planner2.py ' + name_folder
    my_command = "python3 " + file_name
    os.system(my_command)
    return

"""
Saves inputs of the planner into a text file
"""
def save_inputs(b_new, vertices, visited, num_robots):
    global time_step
    line1 = str(b_new) + '\n'
    f = open(abspath + '/planner/inputs.txt', 'w')
    f.write(line1)
    f.write(str(3) + '\n')
    for i in range(3):
        f.write(str(vertices[i]) + '\n')
    f.write(str(visited) + '\n')
    f.write(str(time_step))
    f.close()

    f = open(abspath + '/planner/allinputs.txt', 'a+')
    f.write('time step: ' + str(time_step) + '\n')
    f.write(line1)
    f.write(str(num_robots) + '\n')
    for i in range(num_robots):
        f.write(str(vertices[i]) + '\n')
    f.write(str(visited) + '\n')
    f.close()
    return

"""
Reads output of the planner from a text file
"""
def unpack_plan(num_points):
    f = open(abspath + '/planner/outputs.txt', 'r')
    belief_vector = ast.literal_eval(f.readline())
    if not belief_vector:
        points = [[-1], [-1], [-1]]
        visited = []
    else:
        points = deque()
        for i in range(num_points):
            points.append(ast.literal_eval(f.readline()))
        visited = ast.literal_eval(f.readline())
    return belief_vector, points, visited

"""
Given current belief vector and position of the robots, 
outputs next sets of goals and updated belief vector.
"""
def optimize_path(belief, vertices, visited, num_robots):
    global time_step
    time_step += 1
    save_inputs(belief, vertices, visited, num_robots)
    call_planner()
    belief_v, vertices, visited = unpack_plan(3)
    return belief_v, vertices, visited

def add_0_str(number):
    """add 0 to the beginning to the string if integer is less than 10"""
    if number < 10:
        # add a zero
        n_str = '0' + str(number)
    else:
        n_str = str(number)
    return n_str

def add_00_str(number):
    """add 0 to the beginning to the string if integer is less than 10"""
    if number < 10:
        # add a zero
        n_str = '00' + str(number)
    elif number < 100:
        n_str = '0' + str(number)
    else:
        n_str = str(number)
    return n_str

"""
Get folder path to save files in
"""
def get_folder(belief_num):
    d = datetime.datetime.today()
    m = d.month
    day = d.day

    day_str = add_0_str(day)
    m_str = add_0_str(m)
    r_str = add_00_str(belief_num)

    # assemble folder name
    name_folder = m_str + day_str + "_" + r_str
    total_path = fpath + "/" + name_folder
    if not os.path.exists(total_path):
        os.mkdir(total_path)
    else:
        print("Directory " + total_path + " already exists")
    return total_path

# Info class for pickle output
class Info: pass

class MoveBaseSeq():
    def __init__(self):
        # initialize the move_base_seq node
        rospy.init_node('move_base_seq')

        # all lists will follow this convention
        # index 0: jackal0, index 1: jackal1, index 2: drone
        self.robots = ['drone', 'jackal0', 'jackal1']
        info.robotIDs = self.robots
        info.died = [[None for i in range(len(self.robots))] for i in range(len(self.robots))]
        self.aliveState = [True, True, True]

        # create action clients
        self.clients_list = []
        rospy.loginfo("Waiting for move_base action server...")
        for robot in self.robots:
            client = actionlib.SimpleActionClient('~/'+str(robot)+'/move_base', MoveBaseAction)
            wait = client.wait_for_server(rospy.Duration(5000))
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
                return
            rospy.loginfo("Connected to move base server for " + str(robot))
            self.clients_list.append(client)


        # formatted list of pose and yaw angle goals
        self.pose_lists = deque()
        self.yawangle_lists = deque()

        # generate lookUp dictionary from graph file
        if not os.path.isfile(graphFile):
            print('graph file does not exist. ')
            rospy.signal_shutdown("Shutting down.")
        temp = (' '.join(r.split()) for r in open(graphFile) if not r[0] == '#')
        lookUp = np.genfromtxt(temp, delimiter=' ', names = True)
        lookUp = sorted(lookUp, key = lambda x: x[0])
        self.lookUp = {}
        for vertex in lookUp:
            self.lookUp[vertex[0]] = [vertex[1], vertex[2]]

        # get victim location and initialize belief vector
        self.belief, self.true_v = victimLocation(len(self.lookUp), inputBelief[listv] if len(inputBelief) else None, inputTrue[listv])
        print ('True location: ', self.true_v);
        print ('Belief', self.belief)


        # list of visited vertices for each robot
        self.visited = [[] for _ in self.robots]

        self.start = rospy.Time.now().secs


        # robots start at vertex 1
        self.belief, goals_list, self.visited_list = optimize_path(self.belief, [1, 1, 1], [1], sum(self.aliveState))
        self.set_goals(goals_list)

        # boolean list to keep track of running state
        self.current_goal_reached = [False for _ in range(len(self.robots))]

        # int list of current goal vertex numbers for each robot
        self.current_goal_vertex = [gl[0] for gl in goals_list]

        # check on robots' progress every second
        # update status of robots as necessary
        self.timer = rospy.Timer(rospy.Duration(1), self.update_goals)

        # self.timeStep_timer = rospy.Timer(rospy.Duration(50), self.check_robot_state)
        self.counter = 0

        # start moving all of the robots
        self.movebase_all()

    # takes in a list of goals outputted by the planner and adds to pose_list and yawangle_list
    def set_goals(self, goals_list):
        temp = {time_step: 
                {'visited': self.visited, 
                'time': rospy.Time.now().secs - self.start, 
                'listv': listv,
                'belief': self.belief, 
                'alive': self.aliveState, 
                'true_v': self.true_v  
                }}

        with open(name_folder + '/moreOutput.pickle', 'ab') as handle:
            pickle.dump(temp, handle)
        # list of list of unvisited vertex for each robot
        for i, goal in enumerate(goals_list):
            if goal[0] < 0:
                self.aliveState[i] = False
                self.current_goal_reached[i] = True
                info.died[i][0] = self.current_goal_vertex[i]
                info.died[i][1] = time_step
                if not info.died[i][2]: info.died[i][2] = "Danger"
        goals_list = [goal for goal in goals_list if goal[0] > 0]
        self.vertex_lists = deque()
        goals_list_index = 0
        for i in range(len(self.robots)):
            if self.aliveState[i]:
                self.vertex_lists.append(deque(goals_list[goals_list_index]))
                goals_list_index += 1
            else: 
                self.vertex_lists.append(deque([-1]))

        first_goals = [[goal[0]] for goal in self.vertex_lists]

        self.pose_lists = deque()
        self.yawangle_lists = deque()
        for i in range(len(self.robots)):
            if self.aliveState[i]:
                rospy.loginfo("Setting goals for " + str(self.robots[i]))
                points_seq = getPathCoordinates(first_goals[i], self.lookUp)
                yaweulerangles_seq = getPathAngles(first_goals[i], self.lookUp)
                quat_seq, pose_seq = self.format_goals(points_seq, yaweulerangles_seq)
                self.pose_lists.append(pose_seq)
                self.yawangle_lists.append(quat_seq)
            else: 
                self.pose_lists.append([-1])
                self.yawangle_lists.append([-1])

    # this should convert the list of goals to proper format to send to move_base
    def format_goals(self, points_seq, yaweulerangles_seq):
        quat_seq, pose_seq = deque(), deque()
        for yawangle in yaweulerangles_seq:
            # Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        index = 0
        for point in points:
            # Exploit n variable to cycle in quat_seq
            pose_seq.append(Pose(Point(*point),quat_seq[index]))
            index += 1

        return quat_seq, pose_seq

    # callback function to keep track of robots' progress
    # when all the goals are reached, call planner function again
    def update_goals(self, timer):
        self.counter += 1
        print('counter: ', self.counter)
        rospy.loginfo("t = " + str(time_step) + ", Current state: " + str(self.current_goal_reached))
        if time_step == simulation_time_step_limit:
            info.time_step = time_step
            info.visited = self.visited
            info.time = rospy.Time.now().secs - self.start
            info.missionStatus = "TIME LIMIT EXCEEDED"
            with open(name_folder + '/output.pickle', 'wb') as handle:
                pickle.dump(info, handle)
            self.shutdown_system()
        if self.counter == time_step_limit: 
            self.deactivate_robots()
            self.counter = 0
        for i in range(len(self.robots)):
            if self.aliveState[i]:
                state = self.clients_list[i].get_state()
                # reached a goal
                if state == GoalStatus.SUCCEEDED and not (self.current_goal_reached[i]):
                    self.visited[i].append(self.current_goal_vertex[i])
                    # if victim found
                    if self.current_goal_vertex[i] == self.true_v:
                        now = rospy.Time.now()
                        msg_pose = rospy.wait_for_message('/' + str(self.robots[i]) + '/ground_truth/state', Odometry,
                                                          timeout=5)
                        info.time_step = time_step
                        info.pose = [msg_pose.pose.pose.position.x, msg_pose.pose.pose.position.y]
                        info.visited = self.visited
                        info.time = now.secs - self.start
                        info.missionStatus = "TARGET FOUND"
                        with open(name_folder + '/output.pickle', 'wb') as handle:
                            pickle.dump(info, handle)

                        rospy.loginfo('FOUND VICTIM')
                        rospy.signal_shutdown('Victim found at time t=' + str(now.secs) + '! ')
                        self.shutdown_system()
                    # finished all of its goals!
                    if len(self.pose_lists[i]) == 0:
                        self.current_goal_reached[i] = True
                    else:
                        self.movebase_client(i) # send next goal

        if all(self.current_goal_reached):
            rospy.loginfo("Finished reaching all goals. ")
            print('Time_step: ', time_step)
            input_vertices = []
            print('Alive: ', self.aliveState)
            for i in range(len(self.robots)): 
                if self.aliveState[i]:
                    input_vertices.append(self.current_goal_vertex[i])
                else: 
                    input_vertices.append(-2)
            self.belief, goals_list, self.visited_list = optimize_path(self.belief, input_vertices, self.visited_list, sum(self.aliveState))
            print("goals_list: ", goals_list)
            self.current_goal_reached = [not self.aliveState[i] for i in range(len(self.robots))]

            self.set_goals(goals_list)
            if (not self.belief) or sum(self.aliveState) == 0:
                info.time_step = time_step
                info.pose = None
                info.visited = self.visited
                info.time = rospy.Time.now().secs - self.start
                info.missionStatus = "ALL ROBOTS DEAD"
                with open(name_folder + '/output.pickle','wb') as handle:
                    pickle.dump(info, handle)
                rospy.loginfo('All robots have finished. ')
                self.shutdown_system()

            # self.timeStep_timer = rospy.Timer(rospy.Duration(100), self.check_robot_state)
            self.movebase_all()
            self.counter = 0
        else:
            rospy.loginfo("Robots still running. ")

    def check_robot_state(self, timer):
        if sum(self.current_goal_reached) != len(self.robots):
            index = [i for i, success in enumerate(self.current_goal_reached) if not success]
            for i in index:
                print(self.robots[i] + ' has finished. ')
                self.aliveState[i] = False
                self.current_goal_reached[i] = True
        self.timeStep_timer.shutdown()
        self.timeStep_timer = rospy.Timer(rospy.Duration(50), self.check_robot_state)

    def deactivate_robots(self):
        if sum(self.current_goal_reached) != len(self.robots):
            index = [i for i, success in enumerate(self.current_goal_reached) if not success]
            for i in index:
                print(self.robots[i] + ' has timed out. ')
                info.died[i][0] = self.current_goal_vertex[i]
                info.died[i][1] = time_step
                if not info.died[i][2]: info.died[i][2] = "Stuck"
                self.aliveState[i] = False
                self.current_goal_reached[i] = True
        return

    def finish_simulation(self, timer):
        print('Simulation time is up. ')
        self.shutdown_system()

    # send next goal to the robot
    def movebase_client(self, i):
        client = self.clients_list[i]
        if self.vertex_lists[i][0] != -1 and len(self.pose_lists[i]) > 0:
            rospy.loginfo("Sending next goal for " + str(self.robots[i]))
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "/map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.pose_lists[i].popleft()
            # update current vertex number
            if len(self.vertex_lists[i]):
                self.current_goal_vertex[i] = self.vertex_lists[i].popleft()
            print('Current goals: ', self.current_goal_vertex)
            client.send_goal(goal)
        elif self.vertex_lists[i][0] == -1:
            rospy.loginfo(str(self.robots[i]) + " is no longer active. ")
            self.aliveState[i] = False

        else:
            rospy.loginfo("Empty goal list for " + str(self.robots[i]))

    # move all of the robots
    def movebase_all(self):
        for i in range(len(self.robots)):
            if self.aliveState[i]:
                self.movebase_client(i)

    def shutdown_system(self):
        rospy.signal_shutdown('Shutting down the simulator. ')
        os.system("killall -9 gazebo")
        os.system("killall -9 gzserver gzclient")
        os.system("rosnode kill -a")


if __name__ == '__main__':
    try:
        info = Info()
        moreInfo = Info()
        fpath = sys.argv[1]
        listv = int(sys.argv[2])
        name_folder = get_folder(listv)
        MoveBaseSeq()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
