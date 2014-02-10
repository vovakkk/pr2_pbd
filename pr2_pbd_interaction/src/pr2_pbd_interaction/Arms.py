import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

#class for moving arms and getting arm states
class Arms:
	def __init__(self):
		pass

	def set_gripper_state(self, arm_index, g_state):
		return True

	def set_arm_mode(self, arm_index, arm_mode):#freese/unfreese
		return True

	def is_executing(self):
		pass

	def start_execution(self, action):
		pass

	def stop_execution(self):
		pass

	def get_ee_state(self, arm_ind):
		pass

	def get_joint_states(self, arm_ind):
		pass

	def convert_from_state(self, ee_state, joint_state):
		pass

