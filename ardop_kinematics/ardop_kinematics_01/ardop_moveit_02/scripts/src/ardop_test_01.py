#!/usr/bin/env python



#imports
import sys 
import copy 
import rospy 
import moveit_commander
import moveit_msgs.msg 
import geometry_msgs.msg


from std_msgs.msg import String

def ardop():
	'''
	1. Setting up node. Node name 'ardop'
	2. Instantiate Robot
	3. Create scene
	4. Instantiate the Group for kinematics
	5. Display planned path and trajectory in RViz (Publish)
	'''
	print "----------- Setting up ARDOP -----------"

	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('ardop', anonymous=True)

	#__init__(self,robot_description='ardop_description_02')

	robot = moveit_commander.RobotCommander()

	scene = moveit_commander.PlanningSceneInterface()

	group = moveit_commander.MoveGroupCommander("right_arm")

	ee_link = group.get_end_effector_link()

	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

	print "----------- Setting up RViz -----------"
	rospy.sleep(10)
	print "----------- Welcome to ARDOP -----------"

	print " Robot Groups: "
	print robot.get_group_names()
	print " Current State: "
	print robot.get_current_state()

	print"---------------------------------"

	'''
	1. Define Pose [x,y,z]
	2. 
	



	pose_target = geometry_msgs.msg.Pose()
	pose_target.orientation.w = 1
	pose_target.position.x = -0.25
	pose_target.position.y = -0.25
	pose_target.position.z = 0.25
	group.set_pose_target(pose_target, ee_link)
	'''

	group.set_random_target()


	plan1 = group.plan()
	#group.setPlanningTime(10)

	print "----------- RViz Display Plan -----------"
	rospy.sleep(10)

	display_trajectory = moveit_msgs.msg.DisplayTrajectory()

	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan1)
	display_trajectory_publisher.publish(display_trajectory)

	print "----------- Displaying in RViz -----------"
	rospy.sleep(5)
	# clearing pose
	#group.clear_pose_targets()
	# get joint states
	group_variable_values = group.get_joint_value_target()
	print "Joint Values: ", group_variable_values
	moveit_commander.roscpp_shutdown()
	print"----------- Stopping -----------"







if __name__=='__main__':
	try:
		ardop()
	except rospy.ROSInterruptException:
		pass






















