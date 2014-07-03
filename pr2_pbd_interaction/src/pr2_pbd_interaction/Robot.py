import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
import yaml
from pr2_mechanism_msgs.srv import SwitchController
from pr2_controllers_msgs.msg import Pr2GripperCommand
from Action import Action
from geometry_msgs.msg import Point, Pose, Quaternion

class Robot:
    '''class for moving arms and getting arm states'''
    RELAXED = 0
    HOLD = 1
    CLOSED = 0
    OPENED = 1
    SIDE_PREFIX = ["r", "l"]

    def __init__(self):
        #right - 0, left - 1
        rospy.loginfo("Initializing moveit")
        moveit_commander.roscpp_initialize(sys.argv)
        '''initialize moveit'''
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arms = [moveit_commander.MoveGroupCommander("right_arm"),
            moveit_commander.MoveGroupCommander("left_arm"),
            moveit_commander.MoveGroupCommander("arms")]

        '''is_exec - is the robot currently executing an action'''
        self.is_exec = False

        rospy.loginfo("Moveit initialized, initializing swich controller")
        switch_controller = 'pr2_controller_manager/switch_controller'
        rospy.wait_for_service(switch_controller)
        self.switch_service = rospy.ServiceProxy(switch_controller,
                                                 SwitchController)

        self.gipper_pubs = [rospy.Publisher('/r_gripper_controller/command', Pr2GripperCommand),
            rospy.Publisher('/l_gripper_controller/command', Pr2GripperCommand)]

    '''set the gripper states'''
    def set_gripper_state(self, arm_index, g_state):
        self.gipper_pubs[arm_index].publish(Pr2GripperCommand(0.1 * g_state,100))
        return True

    '''freze/relax the arms'''
    def set_arm_mode(self, arm_index, arm_mode):#freese/unfreese
        controller_name = Robot.SIDE_PREFIX[arm_index] + '_arm_controller'
        if (arm_mode == Robot.RELAXED):
            self.switch_service([], [controller_name], 1)
        else:
            self.switch_service([controller_name], [], 1)
        return True

    '''is the robot currently executing an action?'''
    def is_executing(self):
        return self.is_exec

    '''move arms to pose synchronously'''
    def _move_to_pose(self, arms):
        # x = [arms[0].pose.position.x,
        #   arms[0].pose.position.y,arms[0].pose.position.z,
        #   arms[0].pose.orientation.x, arms[0].pose.orientation.y,
        #   arms[0].pose.orientation.z, arms[0].pose.orientation.w]
        # rospy.loginfo(x)
        # self.arms[0].set_pose_target(x)
        # self.arms[0].go()

        self.arms[0].set_pose_target(arms[0])
        plan1 = self.arms[0].plan()
        self.arms[1].set_pose_target(arms[1])
        plan2 = self.arms[1].plan()

        # self.arms[0].go()
        # self.arms[1].go()
        '''generate plan for each arm'''
        endPose1 = plan1.joint_trajectory.points[len(
            plan1.joint_trajectory.points) - 1]
        endPose2 = plan2.joint_trajectory.points[len(
            plan2.joint_trajectory.points) - 1]
        endPoses = {}
        '''combine plan for both arms'''
        for i in range(0, len(plan1.joint_trajectory.joint_names) - 1):
            endPoses[plan1.joint_trajectory.joint_names[i]] = endPose1.positions[i]
            endPoses[plan2.joint_trajectory.joint_names[i]] = endPose2.positions[i]
        
        # rospy.loginfo(endPoses)
        '''EXECUTE!!!'''
        self.arms[2].set_joint_value_target(endPoses)
        self.arms[2].go()

    '''execute a trajectory synchronously'''
    def _execute_trajectory(self, poses):
        for a_ind in [0, 1]:
            '''speed up the execution, since each pose is calculated seperately'''
            def speedUp(plan):
                new_plan = moveit_msgs.msg.RobotTrajectory()
                new_plan.joint_trajectory = trajectory_msgs.msg.JointTrajectory()
                new_plan.joint_trajectory.points = map(
                    lambda t: trajectory_msgs.msg.JointTrajectoryPoint(
                        map(lambda i: i, t[1].positions),
                        map(lambda vel: vel * 20, t[1].velocities),
                        map(lambda a: a * 20, t[1].accelerations),
                        map(lambda i: i, t[1].effort),
                        t[1].time_from_start / 2)
                        , filter(lambda (i, v): i % 10 == 0, enumerate(
                        plan.joint_trajectory.points)))
                new_plan.joint_trajectory.joint_names = map(lambda i: i, 
                    plan.joint_trajectory.joint_names)
                return new_plan
            plan = self.arms[a_ind].compute_cartesian_path([p[a_ind].pose for p in poses],
                1, 100)[0]
            self.arms[a_ind].execute(speedUp(plan))
            
    '''execute the action (private method)'''
    def _execute_action(self, action, world):
        if (self.is_exec):
            if (action.scan_code == Action.SCAN):
                world.scan_landmarks()
            if (action.scan_code == Action.SCAN_MOVE_ARMS):
                self.hands_up()
                world.scan_landmarks()
                self.hands_down()
            if (action.type == Action.ACTION_QUEUE):
                for act in action.actions:
                    self._execute_action(act, world)
            elif (action.type == Action.POSE):
                best_landmarks = map(world.find_landmark, action.landmark_types)
                rospy.loginfo("--------------------")
                rospy.loginfo(yaml.dump(best_landmarks[0]))
                rospy.loginfo("--------------------")
                def mod_pose((pose, mark)):
                    return Pose(Point(
                        pose.pose.position.x + mark.pose.position.x,
                        pose.pose.position.y + mark.pose.position.y,
                        pose.pose.position.z + mark.pose.position.z),
                        pose.pose.orientation)
                target_pose = map(mod_pose, zip(action.arms, best_landmarks))
                rospy.loginfo(target_pose[0])
                self._move_to_pose(target_pose)
            elif (action.type == Action.GRIPPER):
                pass
            elif (action.type == Action.TRAJECTORY):
                self._execute_trajectory(action.poses)

    '''start executing the action'''
    def start_execution(self, action, world):
        rospy.loginfo("executing...")
        self.is_exec = True
        self._execute_action(action, world)
        self.is_exec = False
        rospy.loginfo("executing complete")

    '''stop executing the action'''
    def stop_execution(self):
        self.is_exec = False
        # self.arms[0].stop()
        # self.arms[1].stop()
        self.arms[2].stop()

    '''get current arm states'''
    def get_arm_state(self):
        return [self.arms[a_ind].get_current_pose() for a_ind in [0, 1]]

    def hands_up(self):
        '''Moves hands out of the way for scanning'''
        self.prev_arms = self.get_arm_state()

        self._move_to_pose([Pose(Point(-0.00324419033606, 
            -0.679973803735, 1.11291513223), 
            Quaternion(0.000267930344083, -0.694507817664, 
            -0.000600285161221, 0.719484856738)),
            Pose(Point(-0.0301325575897, 0.678559678391,
            1.16125279877), Quaternion(-0.000332520531099,
            -0.746892672903, 0.000852050276396, 0.664943981554))])

    def hands_down(self):
        self._move_to_pose(self.prev_arms)
