#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import copy
from geometry_msgs.msg import Pose

# Initialize ROS and MoveIt
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("ur10e_pick_test", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

# Get current pose
start_pose = group.get_current_pose().pose

# Step 1: Move above pick position
above_pick = copy.deepcopy(start_pose)
above_pick.position.x = 0.4
above_pick.position.y = 0.0
above_pick.position.z = 0.3
group.set_pose_target(above_pick)
group.go(wait=True)

# Step 2: Move down to pick position
pick_pose = copy.deepcopy(above_pick)
pick_pose.position.z -= 0.1  # go down
group.set_pose_target(pick_pose)
group.go(wait=True)

# Step 3: Move back up after pick
group.set_pose_target(above_pick)
group.go(wait=True)

# Clear
group.clear_pose_targets()
group.stop()
