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


display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

#### get basic information of robot

plan_frame = move_group.get_planning_frame()
print('planning_frame  =',plan_frame)

eff_link = move_group.get_end_effector_link()

group_names = robot.get_group_names()
print('Group Names  =',group_names)

current_state = robot.get_current_state()
print('current state   =',current_state)

waypoints = []
scale = 1.0
wpose = move_group.get_current_pose().pose
wpose.position.z  += scale * 0.2           # up
wpose.position.y  += scale * 0.1           # sideways

waypoints.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.2
waypoints.append(copy.deepcopy(wpose))

#wpose.position.y += scale * 0.2
#waypoints.append(copy.deepcopy(wpose))

(plan , fraction) = move_group.compute_cartesian_path(waypoints,0.01,0.0)

display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)

display_trajectory_pub.publish(display_trajectory)

move_group.execute(plan, wait=True)

move_group.stop()
move_group.clear_pose_targets()

rospy.sleep(0.1)