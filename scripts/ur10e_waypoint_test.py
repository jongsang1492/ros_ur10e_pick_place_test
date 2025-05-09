#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
from geometry_msgs.msg import Pose

# Initialize ROS and MoveIt
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("ur10e_waypoint_test", anonymous=True)

# Robot, scene, and group setup
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

# Waypoint list
waypoints = []

# Start pose
start_pose = group.get_current_pose().pose
waypoints.append(copy.deepcopy(start_pose))

# Up
pose1 = copy.deepcopy(start_pose)
pose1.position.z += 0.1
waypoints.append(copy.deepcopy(pose1))

# Forward
pose2 = copy.deepcopy(pose1)
pose2.position.x += 0.2
waypoints.append(copy.deepcopy(pose2))

# Down
pose3 = copy.deepcopy(pose2)
pose3.position.z -= 0.1
waypoints.append(copy.deepcopy(pose3))

# ✅ Debug: Print types to make sure they're Pose
for i, w in enumerate(waypoints):
    print(f"[DEBUG] waypoint {i}: {type(w)}")

# Compute Cartesian path
try:
    plan, fraction = group.compute_cartesian_path(
        waypoints,
        0.01,     # eef_step
        0.0      # jump_threshold
    )
except Exception as e:
    print("[ERROR] compute_cartesian_path failed:", e)
    exit(1)

# Report planning result
print(f"Path planning success rate: {fraction * 100:.2f}%")

# Execute if successful
if fraction > 0.9:
    group.execute(plan, wait=True)
else:
    rospy.logwarn("Trajectory not fully planned. Skipping execution.")

group.clear_pose_targets()
group.stop()

