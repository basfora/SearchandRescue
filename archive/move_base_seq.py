#!/usr/bin/env python
import rospy
import math
import numpy as np
import os

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler



class MoveBaseSeq():
    # only change these 
    global graphFile, x_off, y_off, rotation_angle
    graphFile = '/home/samhong/Desktop/Research/SearchandRescue/Scripts/SchoolGraph.txt'
    x_off, y_off = -34.85, -23
    rotation_angle = 0

    # --- helper functions to extract points from a txt file --- #

    # given a point on the floor plan, return a transformed x, y
    def change_coordinates(self, points):
        x, y = points
        theta = np.radians(rotation_angle)
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s, x_off), (s, c, y_off), (0, 0, 1)))

        temp = np.array((x, y, 1))
        output = np.dot(R, temp)
        return [round(i, 3) for i in output]

    # get transformed x, y coordinate of a given node
    def getCoordinates(self, node, lookUp): 
        row = lookUp[node - 1]
        transformed_point = self.change_coordinates(np.array((row[1], row[2])))
        return [transformed_point[0], transformed_point[1], 0]

    # get yaw angle (0 for now)
    def getAngles(self, node, lookUp):
        # row = lookUp[node - 1]
        # return '{}'.format(row[4])
        return [0]

    # given a path (list of nodes), return a list of transformed x, y coordinates
    def getPathCoordinates(self, path, lookUp):
        temp = []
        for i in path:
            temp += self.getCoordinates(i, lookUp)
        return temp

    # given a path (list of nodes), return a list of yaw angles
    def getPathAngles(self, path, lookUp): 
        temp = []
        for i in path:
            temp += self.getAngles(i, lookUp) 
        return temp
    # ---------------------------------------------------------- #
    def __init__(self):
        rospy.init_node('move_base_seq')

        # extract parameters in the .launch file
        # type: list[float]
        points_seq = rospy.get_param('move_base_seq/p_seq')
        yaweulerangles_seq = rospy.get_param('move_base_seq/yea_seq')

        if not os.path.isfile(graphFile):
            rospy.signal_shutdown("Shutting down.")

        quat_seq = list()
        self.pose_seq = list()
        self.goal_cnt = 0
        self.goal_total_cnt = 0
        self.waiting_nav_goals = []

        quat_seq, self.pose_seq = self.get_goals(points_seq, yaweulerangles_seq)

        # generate lookUp 
        temp = (r for r in open(graphFile) if not r[0] == '#')
        lookUp = np.genfromtxt(temp, delimiter=' ', names = True)
        self.lookUp = sorted(lookUp, key = lambda x: x[0])
        
        # create action client
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

        # figure out which robot the script is controlling
        self.robotID = 0
        rospy.loginfo('Enter 0 for jackal0, 1 for jackal1, 2 for drone. ')
        valid = True
        while (valid):
            try: 
                user_input = raw_input()
                if user_input in '012' and len(user_input) == 1:
                    self.robotID = int(user_input)
                    valid = False
                else: 
                    rospy.loginfo ("format error, try again. ")
            except (KeyboardInterrupt, SystemExit): 
                rospy.signal_shutdown("Shutting down.")
                valid = False

        # start moving 
        self.movebase_client()

    # given a sequence of points and yaw angles (list form), return Quaternion and Pose lists
    def get_goals(self, points_seq, yaweulerangles_seq):
        quat_seq, pose_seq = list(), list()
        for yawangle in yaweulerangles_seq:
            # Unpacking the quaternion list and passing it as arguments to Quaternion message constructor
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle*math.pi/180, axes='sxyz'))))
        n = 3
        # Returns a list of lists [[point1], [point2],...[pointn]]
        points = [points_seq[i:i+n] for i in range(0, len(points_seq), n)]
        for point in points:
            # Exploit n variable to cycle in quat_seq
            pose_seq.append(Pose(Point(*point),quat_seq[n-3]))
            n += 1

        return quat_seq, pose_seq

    def active_cb(self):
        rospy.loginfo("Goal pose "+str(self.goal_total_cnt+1)+" is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        # To print current pose at each feedback:
        # rospy.loginfo("Feedback for goal "+str(self.goal_cnt)+": "+str(feedback))
        rospy.loginfo("Feedback for goal pose "+str(self.goal_total_cnt+1)+" received")

    def done_cb(self, status, result):
        self.goal_cnt += 1
        self.goal_total_cnt += 1
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_total_cnt)+" received a cancel request after it started executing, completed execution!")

        if status == 3:
            rospy.loginfo("Goal pose "+str(self.goal_total_cnt)+" reached") 
            if self.goal_cnt< len(self.pose_seq):
                self.movebase_client() 
            else:
                rospy.loginfo("Final goal pose reached!")

                # waiting goals list is empty, see if you need to input more
                if not self.waiting_nav_goals:
                    new_goals = self.getNewGoals()
                    new_goals = new_goals[self.robotID]
                    if new_goals == [-1]:
                        rospy.signal_shutdown("Shutting down.")
                    else: 
                        self.waiting_nav_goals += new_goals        

                next_goal = self.waiting_nav_goals.pop(0)

                quat_seq, self.pose_seq = list(), list()
                self.goal_cnt = 0
                points_seq = self.getPathCoordinates([next_goal], self.lookUp)
                yaweulerangles_seq = self.getPathAngles([next_goal], self.lookUp)
                    
                quat_seq, self.pose_seq = self.get_goals(points_seq, yaweulerangles_seq)
                self.movebase_client()


        if status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_total_cnt)+" was aborted by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_total_cnt)+" aborted, shutting down!")
            return

        if status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_total_cnt)+" has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose "+str(self.goal_total_cnt)+" rejected, shutting down!")
            return

        if status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_total_cnt)+" received a cancel request before it started executing, successfully cancelled!")

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now() 
        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose "+str(self.goal_total_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def getNewGoals(self):
        # to signify finished, return [[-1], [-1], [-1]]
        new_goals = [[53, 38, 36], [43, 41, 37], [43, 41, 37]]
        return new_goals


if __name__ == '__main__':
    try:
        MoveBaseSeq()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")

