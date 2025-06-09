#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import copy
from geometry_msgs.msg import Pose, PoseStamped

# Initialize ROS and MoveIt
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("ur10e_pick_place_test", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

eef_link = group.get_end_effector_link()

rospy.sleep(1.0)

# STEP 1: ADD PICK BOX TO SCENE 
box_pose = PoseStamped()
box_pose.header.frame_id = robot.get_planning_frame()
box_pose.pose.orientation.w = 1.0
box_pose.pose.position.x = 0.4
box_pose.pose.position.y = 0.0
box_pose.pose.position.z = 0.1  

box_name = "pick_object"
scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))

rospy.loginfo("Waiting for box to appear...")
rospy.sleep(1.0)

if box_name in scene.get_known_object_names():
    rospy.loginfo("Box added to scene.")
else:
    rospy.logwarn("Box not added.")

# STEP 2: PICK MOTION 

start_pose = group.get_current_pose().pose

above_pick = copy.deepcopy(start_pose)
above_pick.position.x = 0.4
above_pick.position.y = 0.0
above_pick.position.z = 0.27
rospy.loginfo("Moving to above pick position...")
group.set_pose_target(above_pick)
group.go(wait=True)

pick_pose = copy.deepcopy(above_pick)
pick_pose.position.z -= 0.1
rospy.loginfo("Moving down to pick position...")
group.set_pose_target(pick_pose)
group.go(wait=True)

# STEP 3: ATTACH OBJECT 
scene.attach_box(eef_link, box_name)
rospy.loginfo("Object attached to end-effector.")
rospy.sleep(1.0)

rospy.loginfo("Lifting after pick...")
group.set_pose_target(above_pick)
group.go(wait=True)

# STEP 4: PLACE MOTION 

above_place = copy.deepcopy(above_pick)
above_place.position.x += 0.3
rospy.loginfo("Moving to above place position...")
group.set_pose_target(above_place)
group.go(wait=True)

place_pose = copy.deepcopy(above_place)
place_pose.position.z -= 0.1
rospy.loginfo("Moving down to place position...")
group.set_pose_target(place_pose)
group.go(wait=True)

# STEP 5: DETACH OBJECT
scene.remove_attached_object(eef_link, name=box_name)
rospy.loginfo("Object detached from end-effector.")
rospy.sleep(1.0)

rospy.loginfo("Lifting after place...")
group.set_pose_target(above_place)
group.go(wait=True)

# Cleanup
group.clear_pose_targets()
group.stop()

