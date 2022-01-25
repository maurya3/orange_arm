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

moveit_commander.roscpp_initialize(['joint_states'])
rospy.init_node('move_group_commander',anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

group_name = "orange_arm"
move_group  = moveit_commander.MoveGroupCommander(group_name)


display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,queue_size=20)

#### get basic information of robot

plan_frame = move_group.get_planning_frame()
print('planning_frame  =',plan_frame)

#eff_link = move_group.get_end_effector_link()

group_names = robot.get_group_names()
print('Group Names  =',group_names)

current_state = robot.get_current_state()
print('current state   =',current_state)

pose_goal = geometry_msgs.msg.Pose()

pose_goal.position.x = 0.0
pose_goal.position.y = -0.12
pose_goal.position.z = 0.97

pose_goal.orientation.w = 0.707
pose_goal.orientation.x = 0.0
pose_goal.orientation.y = 0.0
pose_goal.orientation.z = 0.0

move_group.set_planner_id("RRTConnectDefault")
move_group.set_pose_target(pose_goal)
move_group.set_planning_time(100)
move_group.go(wait=True)
print('done')
move_group.stop()
move_group.clear_pose_targets()

rospy.sleep(0.1)
