import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from pr2_mechanism_msgs.srv import SwitchController
from pr2_controllers_msgs.msg import Pr2GripperCommand
from Action import Action

#class for moving arms and getting arm states
class Robot:
	RELAXED = 0
	HOLD = 1
	CLOSED = 0
	OPENED = 1
	SIDE_PREFIX = ["r", "l"]

	def __init__(self):
		#right - 0, left - 1
		rospy.loginfo("Initializing moveit")
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.arms = [moveit_commander.MoveGroupCommander("right_arm"),
			moveit_commander.MoveGroupCommander("left_arm"),
			moveit_commander.MoveGroupCommander("arms")]


		self.is_exec = False

		rospy.loginfo("Moveit initialized, initializing swich controller")
		switch_controller = 'pr2_controller_manager/switch_controller'
		rospy.wait_for_service(switch_controller)
		self.switch_service = rospy.ServiceProxy(switch_controller,
												 SwitchController)

		self.gipper_pubs = [rospy.Publisher('/r_gripper_controller/command', Pr2GripperCommand),
			rospy.Publisher('/l_gripper_controller/command', Pr2GripperCommand)]

	def set_gripper_state(self, arm_index, g_state):
		self.gipper_pubs[arm_index].publish(Pr2GripperCommand(0.1 * g_state,100))
		return True

	def set_arm_mode(self, arm_index, arm_mode):#freese/unfreese
		controller_name = Robot.SIDE_PREFIX[arm_index] + '_arm_controller'
		if (arm_mode == Robot.RELAXED):
			self.switch_service([], [controller_name], 1)
		else:
			self.switch_service([controller_name], [], 1)
		return True

	def is_executing(self):
		return self.is_exec

	def _move_to_pose(self, arms):
		targs = [None, None]
		for a_ind in [0, 1]:
			pos = arms[a_ind]['position']
			ori = arms[a_ind]['orientation']
			targs[a_ind] = [pos['x'], pos['y'], pos['z'], ori['x'], ori['y'], ori['z'], ori['w']]

		self.arms[0].set_pose_target(targs[0])
		plan1 = self.arms[0].plan()
		self.arms[1].set_pose_target(targs[1])
		plan2 = self.arms[1].plan()
		endPose1 = plan1.joint_trajectory.points[len(
			plan1.joint_trajectory.points) - 1]
		endPose2 = plan2.joint_trajectory.points[len(
			plan2.joint_trajectory.points) - 1]
		endPoses = {}
		for i in range(0, len(plan1.joint_trajectory.joint_names) - 1):
			endPoses[plan1.joint_trajectory.joint_names[i]] = endPose1.positions[i]
			endPoses[plan2.joint_trajectory.joint_names[i]] = endPose2.positions[i]
		
		self.arms[2].set_joint_value_target(endPoses)
		self.arms[2].go()

	def _execute_action(self, action):
		if (self.is_exec):
			if (action.type == Action.ACTION_QUEUE):
				for act in action.actions:
					self._execute_action(act)
			elif (action.type == Action.POSE):
				self._move_to_pose(action.arms)
			elif (action.type == Action.GRIPPER):
				pass
			elif (action.type == Action.TRAJECTORY):
				cur_pose = self.get_arm_state()
				for pose in action.poses:
					self._move_to_pose([{
							"position" : {
								"x" : cur_pose[a_ind]["position"]["x"] + 
									pose["arms"][a_ind]["position"]["x"],
								"y" : cur_pose[a_ind]["position"]["y"] + 
									pose["arms"][a_ind]["position"]["y"],
								"z" : cur_pose[a_ind]["position"]["z"] + 
									pose["arms"][a_ind]["position"]["z"]
							}, "orientation" : {
								"x" : pose["arms"][a_ind]["orientation"]["x"],
								"y" : pose["arms"][a_ind]["orientation"]["y"],
								"z" : pose["arms"][a_ind]["orientation"]["z"],
								"w" : pose["arms"][a_ind]["orientation"]["w"]
							} 
						} for a_ind in [0, 1]])

	def start_execution(self, action):
		rospy.loginfo("executing...")
		self.is_exec = True
		self._execute_action(action)
		self.is_exec = False
		rospy.loginfo("executing complete")

	def stop_execution(self):
		self.is_exec = False
		# self.arms[0].stop()
		# self.arms[1].stop()
		self.arms[2].stop()

	def get_arm_state(self):
		states = [None, None]
		for a_ind in [0, 1]:
			pose = self.arms[a_ind].get_current_pose()
			pos = pose.pose.position
			ori = pose.pose.orientation
			states[a_ind] = {
					"position" : {
						"x" : pos.x,
						"y" : pos.y,
						"z" : pos.z
					}, "orientation" : {
						"x" : ori.x,
						"y" : ori.y,
						"z" : ori.z,
						"w" : ori.w
					} 
				}
		return states

