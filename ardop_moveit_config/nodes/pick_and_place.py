import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

# Joint poses
ARM_DEFAULT = [0]*6
GRIPPER_DEFAULT = [0]
GRIPPER_OPEN = [1]
GRIPPER_CLOSE = [-0.5]

# Setup node
print "\nStarting setup"
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python', anonymous=True)

# Setup move group
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 
	moveit_msgs.msg.DisplayTrajectory)

# Setup arm group
arm_group = moveit_commander.MoveGroupCommander("arm")
arm_group.set_goal_tolerance(0.02)
arm_group.set_planning_time(5)
arm_group.set_num_planning_attempts(10)
end_effector_link =  arm_group.get_end_effector_link()

# Setup end effector group
gripper_group = moveit_commander.MoveGroupCommander("gripper")
gripper_group.set_goal_tolerance(0.02)
gripper_group.set_planning_time(1)
gripper_group.set_num_planning_attempts(5)

# Display robot info
print "\nReference frame: %s" % arm_group.get_planning_frame()
print "\nRobot Groups:"
print robot.get_group_names()

def execute_plan(plan, group=arm_group):
	print "\n>>> Visualizing plan"

	# Waiting while Rviz displays the plan
	rospy.sleep(1)
	
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan)
	display_trajectory_publisher.publish(display_trajectory);

	# Waiting while plan is visualized (again)
	rospy.sleep(1)

	print "\n>>> Executing plan"
	group.go(wait=True)

def move_arm(x=0, y=0, z=0, rx=None, ry=None, rz=None, rw=None, pose=None):
	print "\nGenerating Inverse Kinematics..."
	
	if pose is not None:
		joint_values = arm_group.get_current_joint_values()
		print "\nJoint Values:", joint_values
		
		if len(joint_values) == len(pose):
			arm_group.set_joint_value_target(pose)
		else:
			print "Pose length doesn't match"
			return False

	else:
		pose_target = geometry_msgs.msg.Pose()

		# Set position
		pose_target.position.x = x
		pose_target.position.y = y
		pose_target.position.z = z

		# Set orientation
		if rx is not None: pose_target.orientation.x = rx
		if ry is not None: pose_target.orientation.y = ry
		if rz is not None: pose_target.orientation.z = rz
		if rw is not None: pose_target.orientation.w = rw
		
		# Set position target
		xyz = [pose_target.position.x, pose_target.position.y, pose_target.position.z]

		# TODO: Implement pose targets
		# print "\nSet Pose:\n", pose_target
		# arm_group.set_pose_target(pose_target, end_effector_link)
		
		print "\nSet Position XYZ:", xyz
		arm_group.set_position_target(xyz, end_effector_link)
	
	# Execute plan
	execute_plan(arm_group.plan())

def move_gripper(pose):
	print "\nMove Gripper..."

	joint_values = gripper_group.get_current_joint_values()
	print "\nJoint Values:", joint_values
	
	if len(joint_values) == len(pose):
		gripper_group.set_joint_value_target(pose)
	else:
		print "Pose length doesn't match"
		return False

	# Execute plan
	execute_plan(gripper_group.plan(), gripper_group)

def run():
	move_gripper(GRIPPER_OPEN)
	move_arm(0.059099, 0.016645, 0.166850)
	move_gripper(GRIPPER_CLOSE)
	move_arm(pose=ARM_DEFAULT)
	move_gripper(GRIPPER_DEFAULT)

	moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
	run()
