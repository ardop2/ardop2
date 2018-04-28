import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# Select forward or inverse here
IK = True

# Enable/disable random pose for IK
RANDOM_POSE = False

# Joint poses
ARM_DEFAULT = [0]*6

print "\nStarting setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("arm")
end_effector_link =  group.get_end_effector_link()

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
	moveit_msgs.msg.DisplayTrajectory)

# print "\nWaiting for RVIZ..."
# rospy.sleep(10)

print "\nReference frame: %s" % group.get_planning_frame()
print "\nRobot Groups:"
print robot.get_group_names()
print "\nCurrent Pose:"
print group.get_current_pose()
# print "\nRobot State:"
# print robot.get_current_state()

group.set_goal_tolerance(0.02)
group.set_planning_time(5)
group.set_num_planning_attempts(10)
# group.allow_replanning(True)

if IK:
	print "\nGenerating Inverse Kinematics..."
	
	pose_target = geometry_msgs.msg.Pose()

	# Set position
	pose_target.position.x = 0.15
	pose_target.position.y = 0
	pose_target.position.z = 0.2

	# Set orientation
	# pose_target.orientation.x = 0.778503111668
	# pose_target.orientation.y = -0.601031377053
	# pose_target.orientation.z = 0.179308816961
	# pose_target.orientation.w = 0.0232924253989
	
	# Set position target
	xyz = [pose_target.position.x, pose_target.position.y, pose_target.position.z]
	
	if RANDOM_POSE:
		random_pose_target = group.get_random_pose()
		print "\nRandom Pose:\n", random_pose_target
		group.set_pose_target(random_pose_target, end_effector_link)
	else:
		print "\nSet Pose:\n", pose_target
		# group.set_pose_target(pose_target, end_effector_link)
		print "\nSet Position XYZ:", xyz
		group.set_position_target(xyz, end_effector_link)
	
	plan = group.plan()

else:
	print "\nGenerating Forward Kinematics..."
	pose_target = geometry_msgs.msg.Pose()
	group_variable_values = group.get_current_joint_values()
	print "\nJoint Values:", group_variable_values
	
	# group_variable_values[3] = 0
	group_variable_values = ARM_DEFAULT
	group.set_joint_value_target(group_variable_values)
	plan = group.plan()

print "\nWaiting while Rviz displays the plan..."
rospy.sleep(1)

print "\n>>> Visualizing plan"
display_trajectory = moveit_msgs.msg.DisplayTrajectory()
display_trajectory.trajectory_start = robot.get_current_state()
display_trajectory.trajectory.append(plan)
display_trajectory_publisher.publish(display_trajectory);

print "\nWaiting while plan is visualized (again)..."
rospy.sleep(1)

print "\n>>> Executing plan"
group.go(wait=True)

moveit_commander.roscpp_shutdown()
