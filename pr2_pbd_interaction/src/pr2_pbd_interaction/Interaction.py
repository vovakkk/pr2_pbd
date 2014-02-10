'''Main interaction loop'''

import roslib
roslib.load_manifest('pr2_pbd_interaction')

#functional code
from functools import partial

# Generic libraries
import rospy
import time
from visualization_msgs.msg import MarkerArray

# Local stuff
# from World import World
from RobotSpeech import RobotSpeech
from Session import Session
from Response import Response
from Arms import Arms
from Action import Action
from ObjectType import ObjectType
from pr2_pbd_interaction.msg import ArmState, GripperState
from pr2_pbd_interaction.msg import Object
from pr2_pbd_interaction.msg import GuiCommand
from pr2_pbd_speech_recognition.msg import Command
from pr2_social_gaze.msg import GazeGoal


class Interaction:
    '''Finite state machine for the human interaction'''

    _is_programming = True
    _is_recording_motion = False
    _arm_trajectory = None

    def __init__(self):
        self.arms = Arms()
        #self.world = World()
        #self.session = Session(object_list=self.world.get_frame_list(),
                               #is_debug=True)
        self.session = Session()
        self._viz_publisher = rospy.Publisher('visualization_marker_array',
                                              MarkerArray)

        rospy.Subscriber('recognized_command', Command, self.speech_command_cb)
        rospy.Subscriber('gui_command', GuiCommand, self.gui_command_cb)

        self._undo_function = None

        self.responses = {
            Command.TEST_MICROPHONE: Response(partial(Interaction.empty_response,
                                [RobotSpeech.TEST_RESPONSE, GazeGoal.NOD])),
            Command.RELAX_RIGHT_ARM: Response(partial(self.relax_arm, 0)),
            Command.RELAX_LEFT_ARM: Response(partial(self.relax_arm, 1)),
            Command.OPEN_RIGHT_HAND: Response(partial(self.open_hand, 0)),
            Command.OPEN_LEFT_HAND: Response(partial(self.open_hand, 1)),
            Command.CLOSE_RIGHT_HAND: Response(partial(self.close_hand, 0)),
            Command.CLOSE_LEFT_HAND: Response(partial(self.close_hand, 1)),
            Command.STOP_EXECUTION: Response(self.stop_execution),
            Command.UNDO: Response(self.undo),
            Command.DELETE_ALL_STEPS: Response(self.delete_all_steps),
            Command.DELETE_LAST_STEP: Response(self.delete_last_step),
            Command.REPEAT_LAST_STEP: Response(self.repeat_step),
            Command.FREEZE_RIGHT_ARM: Response(partial(self.freeze_arm, 0)),
            Command.FREEZE_LEFT_ARM: Response(partial(self.freeze_arm, 1)),
            Command.CREATE_NEW_ACTION: Response(self.create_action),
            Command.EXECUTE_ACTION: Response(self.execute_action),
            Command.NEXT_ACTION: Response(self.next_action),
            Command.PREV_ACTION: Response(self.previous_action),
            Command.SAVE_POSE: Response(self.save_step),
            Command.RECORD_OBJECT_POSE: Response(self.record_object_pose),
            Command.START_RECORDING_MOTION: Response(
                                            self.start_recording),
            Command.STOP_RECORDING_MOTION: Response(self.stop_recording),
            Command.SAVE_ACTION: Response(self.save_experiment_state)}

        rospy.loginfo('Interaction initialized.')

    def open_hand(self, arm_index):
        '''Opens gripper on the indicated side'''
        if self.arms.set_gripper_state(arm_index, GripperState.OPEN):
            speech_response = Response.open_responses[arm_index]
            if (Interaction._is_programming):
                self.save_gripper_step(arm_index, GripperState.OPEN)
                speech_response = (speech_response + ' ' +
                                   RobotSpeech.STEP_RECORDED)
            return [speech_response, Response.glance_actions[arm_index]]
        else:
            return [Response.already_open_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def close_hand(self, arm_index):
        '''Closes gripper on the indicated side'''
        if Arms.set_gripper_state(arm_index, GripperState.CLOSED):
            speech_response = Response.close_responses[arm_index]
            if (Interaction._is_programming):
                self.save_gripper_step(arm_index, GripperState.CLOSED)
                speech_response = (speech_response + ' ' +
                                   RobotSpeech.STEP_RECORDED)
            return [speech_response, Response.glance_actions[arm_index]]
        else:
            return [Response.already_closed_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def relax_arm(self, arm_index):
        '''Relaxes arm on the indicated side'''
        if self.arms.set_arm_mode(arm_index, ArmMode.RELEASE):
            return [Response.release_responses[arm_index],
                    Response.glance_actions[arm_index]]
        else:
            return [Response.already_released_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def freeze_arm(self, arm_index):
        '''Stiffens arm on the indicated side'''
        if self.arms.set_arm_mode(arm_index, ArmMode.HOLD):
            return [Response.hold_responses[arm_index],
                    Response.glance_actions[arm_index]]
        else:
            return [Response.already_holding_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def edit_action(self):
        '''Goes back to edit mode'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                return [RobotSpeech.ALREADY_EDITING, GazeGoal.SHAKE]
            else:
                Interaction._is_programming = True
                return [RobotSpeech.SWITCH_TO_EDIT_MODE, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def save_action(self):
        '''Goes out of edit mode'''
        self.session.save_current_action()
        #Interaction._is_programming = False
        return [RobotSpeech.ACTION_SAVED + ' ' +
                str(self.session.current_action_index), GazeGoal.NOD]

    def create_action(self):
        '''Creates a new empty action'''
        self.world.clear_all_objects()
        self.session.new_action()
        Interaction._is_programming = True
        return [RobotSpeech.SKILL_CREATED + ' ' +
                str(self.session.current_action_index), GazeGoal.NOD]

    def next_action(self):
        '''Switches to next action'''
        if self.session.next_action():
            return [RobotSpeech.SWITCH_SKILL + ' ' +
                    str(self.session.current_action_index), GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NEXT_SKILL + ' ' +
                    str(self.session.current_action_index), GazeGoal.SHAKE]

    def previous_action(self):
        '''Switches to previous action'''
        if self.session.previous_action():
            return [RobotSpeech.SWITCH_SKILL + ' ' +
                    str(self.session.current_action_index), GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_PREV_SKILL + ' ' +
                    str(self.session.current_action_index), GazeGoal.SHAKE]

    def delete_last_step(self):
        '''Deletes last step of the current action'''
        if (Interaction._is_programming):
            self.session.delete_last_step()
            self._undo_function = self._resume_last_step
            return [RobotSpeech.LAST_POSE_DELETED, GazeGoal.NOD]
        else:
            return ['Action ' + str(self.session.current_action_index) +
                    RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]

    def delete_all_steps(self):
        '''Deletes all steps in the current action'''
        if (Interaction._is_programming):
            self.session.clear_current_action()
            self._undo_function = self._resume_all_steps
            return [RobotSpeech.SKILL_CLEARED, GazeGoal.NOD]
        else:
            return ['Action ' + str(self.session.current_action_index) +
                    RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
    
    def repeat_step(self):
        '''copies previous step'''
        if (Interaction._is_programming):
            if self.session.n_frames() > 0:
                self.session.repeat_step()
                self._undo_function = self.delete_last_step
                return [RobotSpeech.LAST_POSE_REPEATED, GazeGoal.NOD]
            else:
                return [RobotSpeech.SKILL_EMPTY, GazeGoal.SHAKE]
        else:
            return ['Action ' + str(self.session.current_action_index) +
                    RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
    
    def undo(self):
        '''Undoes the effect of the previous command'''
        if (self._undo_function == None):
            return [RobotSpeech.ERROR_NOTHING_TO_UNDO, GazeGoal.SHAKE]
        else:
            return self._undo_function()

    def _resume_all_steps(self):
        '''Resumes all steps after clearing'''
        self.session.undo_clear()
        return [RobotSpeech.ALL_POSES_RESUMED, GazeGoal.NOD]

    def _resume_last_step(self):
        '''Resumes last step after deleting'''
        self.session.resume_deleted_step()
        return [RobotSpeech.POSE_RESUMED, GazeGoal.NOD]

    def stop_execution(self):
        '''Stops ongoing execution'''
        if (self.arms.is_executing()):
            self.arms.stop_execution()
            return [RobotSpeech.STOPPING_EXECUTION, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_EXECUTION, GazeGoal.SHAKE]

    def save_gripper_step(self, arm_index, gripper_state):
        '''Saves an action step that involves a gripper state change'''
        if (Interaction._is_programming):
            act_step = Action()
            act_step.type = Action.GRIPPER
            act_step.gripper_state = gripper_state
            act_step.arm_index = arm_index
            self.session.add_step_to_action(act_step)
            #states = self._get_arm_states()
            #step = ActionStep()
            #step.type = ActionStep.ARM_TARGET
            #step.armTarget = ArmTarget(states[0], states[1], 0.2, 0.2)
            #actions = [self.arms.get_gripper_state(0),
                        #self.arms.get_gripper_state(1)]
            #actions[arm_index] = gripper_state
            #step.gripperAction = GripperAction(actions[0], actions[1])
            #self.session.add_step_to_action(step,
                                            #self.world.get_frame_list())

    def start_recording(self):
        '''Starts recording continuous motion'''
        if (Interaction._is_programming):
            if (not Interaction._is_recording_motion):
                Interaction._is_recording_motion = True
                #Interaction._arm_trajectory = ArmTrajectory()
                Interaction._arm_trajectory = []
                                
                return [RobotSpeech.STARTED_RECORDING_MOTION,
                        GazeGoal.NOD]
            else:
                return [RobotSpeech.ALREADY_RECORDING_MOTION,
                        GazeGoal.SHAKE]
        else:
            return ['Action ' + str(self.session.current_action_index) +
                    RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]

    def stop_recording(self):
        '''Stops recording continuous motion'''
        if ((Interaction._is_recording_motion) and (len(Interaction._arm_trajectory) > 0)):
            Interaction._is_recording_motion = False
            traj_step = Action()
            traj_step.type = Action.TRAJECTORY

            waited_time = Interaction._arm_trajectory[0]["timing"]
            _relative_motion_start = Interaction._arm_trajectory[0]["arms"]
            traj_step.poses = [{ "timing" : (pose["timing"]
                                - waited_time + rospy.Duration(0.1)).to_nsec(),
                                "arms" : [{ "position" : {
                                    "x" : pose["arms"][a_ind]
                                        .ee_pose.position.x -
                                        _relative_motion_start[a_ind].ee_pose.position.x,
                                    "y" : pose["arms"][a_ind]
                                        .ee_pose.position.y -
                                        _relative_motion_start[a_ind].ee_pose.position.y,
                                    "z" : pose["arms"][a_ind]
                                        .ee_pose.position.z -
                                        _relative_motion_start[a_ind].ee_pose.position.z
                                }, "orientation" : { 
                                    "x" : pose["arms"][a_ind]
                                        .ee_pose.orientation.x,
                                    "y" : pose["arms"][a_ind]
                                        .ee_pose.orientation.y,
                                    "z" : pose["arms"][a_ind]
                                        .ee_pose.orientation.z,
                                    "w" : pose["arms"][a_ind]
                                        .ee_pose.orientation.w
                                }, "joints" : pose["arms"][a_ind].joint_pose } 
                                for a_ind in [0, 1] ] } for pose 
                                in Interaction._arm_trajectory]
                                                    
            self.session.add_step_to_action(traj_step)
            
            
            Interaction._arm_trajectory = None
            
            return [RobotSpeech.STOPPED_RECORDING_MOTION + ' ' +
                    RobotSpeech.STEP_RECORDED, GazeGoal.NOD]
        else:
            return [RobotSpeech.MOTION_NOT_RECORDING, GazeGoal.SHAKE]

    def _save_arm_to_trajectory(self):
        '''Saves current arm state into continuous trajectory'''
        if (Interaction._is_recording_motion):
            Interaction._arm_trajectory.append({
                "arms" : map(lambda a_ind:
                                ArmState(ArmState.ROBOT_BASE,
                                        Arms.get_ee_state(a_ind),
                                        Arms.get_joint_state(a_ind), Object()), [0, 1]),
                "timing" : rospy.Time.now()})

    def save_step(self):
        '''Saves current arm state as an action step'''
        if (Interaction._is_programming):
            step = Action()
            step.type = Action.POSE
            step.arms = self._get_arm_states()
            step.target = ObjectType(0)
            self.session.add_step_to_action(step)
            return [RobotSpeech.STEP_RECORDED, GazeGoal.NOD]
        else:
            return ['Action ' + str(self.session.current_action_index) +
                    RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]

    def _get_arm_states(self):
        '''Returns the current arms states in the right format'''
        abs_ee_poses = map(Arms.get_ee_state, [0, 1])
        joint_poses = map(Arms.get_joint_state, [0, 1])
                
        states = [None, None]

        for arm_index in [0, 1]:
            nearest_obj = None#self.world.get_nearest_object(
                               #                 abs_ee_poses[arm_index])

            if (nearest_obj == None):
                states[arm_index] = Arms.convert_from_state(abs_ee_poses[arm_index],
                    joint_poses[arm_index])
            else:
                # Relative
                rel_ee_pose = World.transform(
                                    abs_ee_poses[arm_index],
                                    'base_link', nearest_obj.name)
                states[arm_index] = ArmState(ArmState.OBJECT,
                                    rel_ee_pose,
                                    joint_poses[arm_index], nearest_obj)
        
        return states

    def execute_action(self):
        '''Starts the execution of the current action'''
        if (len(self.session.actions) > 0):
            if (self.session.n_frames() > 0):
                #self.session.save_current_action()
                action = self.session.get_current_action()
                self.arms.start_execution(action)
                #if (action.is_object_required()):
                    #if (self.world.update_object_pose()):
                        #self.session.get_current_action().update_objects(
                                                #self.world.get_frame_list())
                        #self.arms.start_execution(action)
                    #else:
                        #return [RobotSpeech.OBJECT_NOT_DETECTED,
                                #GazeGoal.SHAKE]
                #else:
                    #self.arms.start_execution(action)

                return [RobotSpeech.START_EXECUTION + ' ' +
                        str(self.session.current_action_index), None]
            else:
                return [RobotSpeech.EXECUTION_ERROR_NOPOSES + ' ' +
                        str(self.session.current_action_index), GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def speech_command_cb(self, command):
        '''Callback for when a speech command is receieved'''
        if command.command in self.responses.keys():
            rospy.loginfo('\033[32m Calling response for command ' +
                          command.command + '\033[0m')
            response = self.responses[command.command]

            if (not self.arms.is_executing()):
                if (self._undo_function != None):
                    response.respond()
                    self._undo_function = None
                else:
                    response.respond()
            else:
                if command.command == Command.STOP_EXECUTION:
                    response.respond()
                else:
                    rospy.logwarn('Ignoring speech command during execution: '
                                  + command.command)
        else:
            switch_command = "switch-to-action "
            name_command = "name-action "
            add_action_commnad = "add-action-step "
            if (switch_command in command.command):
                action_name = command.command[
                                len(switch_command):len(command.command)]
                if (self.session.n_actions() > 0):
                    self.session.switch_to_action_by_name(action_name)
                    response = Response(partial(Interaction.empty_response,
                        [RobotSpeech.SWITCH_SKILL + action_name,
                         GazeGoal.NOD]))
                else:
                    response = Response(partial(Interaction.empty_response,
                        [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]))
                response.respond()
            elif (name_command in command.command):
                action_name = command.command[
                                len(name_command):len(command.command)]
                                
                if (len(action_name) > 1):
                    self.session.name_action(action_name)
                    Response(partial(Interaction.empty_response,
                            [RobotSpeech.SWITCH_SKILL + action_name,
                            GazeGoal.NOD])).respond()
                else:
                    Response(partial(Interaction.empty_response,
                        [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE])).respond()
            elif (add_action_commnad in command.command):
                action_name = command.command[
                                len(add_action_commnad):len(command.command)]
                if (self.session.add_action_step_action(action_name)):
                    Response(partial(Interaction.empty_response,
                            [RobotSpeech.SWITCH_SKILL + action_name,
                            GazeGoal.NOD])).respond()
                else:
                    Response(partial(Interaction.empty_response,
                        [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE])).respond()
                    
            else:
                rospy.logwarn('\033[32m This command (' + command.command
                              + ') is unknown. \033[0m')

    def gui_command_cb(self, command):
        '''Callback for when a GUI command is received'''
        if (not self.arms.is_executing()):
            if (command.command == GuiCommand.SWITCH_TO_ACTION):
                action_no = command.param
                self.session.switch_to_action(action_no)
                response = Response(partial(Interaction.empty_response,
                    [RobotSpeech.SWITCH_SKILL + str(action_no),
                        GazeGoal.NOD]))
                response.respond()
            elif (command.command == GuiCommand.SELECT_ACTION_STEP):
                step_no = command.param
                self.session.select_action_step(step_no)
                rospy.loginfo('Selected action step ' + str(step_no))
            else:
                rospy.logwarn('\033[32m This command (' + command.command
                                + ') is unknown. \033[0m')
        else:
            rospy.logwarn('Ignoring GUI command during execution: ' +
                                command.command)

    def update(self):
        '''General update for the main loop'''
        # self.arms.update()

        # if (self.arms.status != ExecutionStatus.NOT_EXECUTING):
        #     if (self.arms.status != ExecutionStatus.EXECUTING):
        #         self._end_execution()
        if (Interaction._is_recording_motion):
            self._save_arm_to_trajectory()

        #is_world_changed = self.world.update()
        #if (self.session.n_actions() > 0):
            #action = self.session.get_current_action()
            #action.update_viz()
            #r_target = action.get_requested_targets(0)
            #if (r_target != None):
                #self.arms.start_move_to_pose(r_target, 0)
                #action.reset_targets(0)
            #l_target = action.get_requested_targets(1)
            #if (l_target != None):
                #self.arms.start_move_to_pose(l_target, 1)
                #action.reset_targets(1)

            #action.delete_requested_steps()

            #states = self._get_arm_states()
            #action.change_requested_steps(states[0], states[1])

            #if (is_world_changed):
                #rospy.loginfo('The world has changed.')
                #self.session.get_current_action().update_objects(
                                        #self.world.get_frame_list())

        time.sleep(0.1)

    # def _end_execution(self):
    #     '''Responses for when the action execution ends'''
    #     if (self.arms.status == ExecutionStatus.SUCCEEDED):
    #         Response.say(RobotSpeech.EXECUTION_ENDED)
    #         Response.perform_gaze_action(GazeGoal.NOD)
    #     elif (self.arms.status == ExecutionStatus.PREEMPTED):
    #         Response.say(RobotSpeech.EXECUTION_PREEMPTED)
    #         Response.perform_gaze_action(GazeGoal.SHAKE)
    #     else:
    #         Response.say(RobotSpeech.EXECUTION_ERROR_NOIK)
    #         Response.perform_gaze_action(GazeGoal.SHAKE)

    #     self.arms.status = ExecutionStatus.NOT_EXECUTING

    def record_object_pose(self):
        '''Makes the robot look for a table and objects'''
        if (self.world.update_object_pose()):
            if (self.session.n_actions() > 0):
                self.session.get_current_action().update_objects(
                                            self.world.get_frame_list())
            return [RobotSpeech.START_STATE_RECORDED, GazeGoal.NOD]
        else:
            return [RobotSpeech.OBJECT_NOT_DETECTED, GazeGoal.SHAKE]

    def save_experiment_state(self):
        '''Saves session state'''
        
        if self.session.save_current_action():
            return [RobotSpeech.ACTION_SAVED, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    @staticmethod
    def empty_response(responses):
        '''Default response to speech commands'''
        return responses
