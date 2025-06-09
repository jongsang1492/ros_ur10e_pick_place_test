#!/usr/bin/env pythonAdd commentMore actions

import rospy
import moveit_commander
import geometry_msgs.msg
import sys

rospy.init_node("ur10e_trajectory_test", anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")  

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 1.0
pose_target.position.x = 0.5
pose_target.position.y = 0.0
pose_target.position.z = 0.5
group.set_pose_target(pose_target)

group.go(wait=True)
group.stop()
group.clear_pose_targets()
