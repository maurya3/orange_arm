from os import wait
import sys
import copy
from moveit_commander import move_group
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import csv
import numpy as np
import math

moveit_commander.roscpp_initialize(['joint_states'])
rospy.init_node('move_group_commander',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "orange_arm"
move_group  = moveit_commander.MoveGroupCommander(group_name)


display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

#### get basic information of robot

plan_frame = move_group.get_planning_frame()
print('planning_frame  =',plan_frame)

#eff_link = move_group.get_end_effector_link()

group_names = robot.get_group_names()
print('Group Names  =',group_names)

current_state = robot.get_current_state()
print('current state   =',current_state)

joint_goals = move_group.get_current_joint_values()
print(joint_goals)
current_pose = move_group.get_current_pose().pose
def main():
    while not rospy.is_shutdown():
        rate = rospy.Rate(110)
    
        current_pose = move_group.get_current_pose().pose
        print(current_pose)
        goal = [0, 0,0,-pi/2,0.0,0.0,0.0]  
        
        joint_goals[0] = goal[0]
        joint_goals[1] = goal[1]
        joint_goals[2] = goal[2]
        joint_goals[3] = goal[3]
        joint_goals[4] = goal[4]
        joint_goals[5] = goal[5]



        move_group.go(joint_goals,wait=True)
        move_group.stop()
        rate.sleep()

    rospy.spin()

if __name__=="__main__":
    main()