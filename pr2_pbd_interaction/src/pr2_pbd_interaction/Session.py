'''Everything related to an experiment session'''

from Action import Action
import rospy
import os
import yaml
from pr2_pbd_interaction.msg import ExperimentState
from pr2_pbd_interaction.srv import GetExperimentState
from pr2_pbd_interaction.srv import GetExperimentStateResponse


class Session:
    '''Class that holds all of the actions'''


    def __init__(self):
        self._exp_number = None
        self._selected_step = 0
        self.object_names = []

        # if (is_debug):
        #     self._exp_number = rospy.get_param(
        #                         '~experimentNumber')
        #     self._data_dir = self._get_data_dir(self._exp_number)
        #     if (not os.path.exists(self._data_dir)):
        #         os.mkdir(self._data_dir)
        # else:
        #     self._get_participant_id()
        # rospy.set_param('data_directory', self._data_dir)

        self.actions = Action.get_saved_actions()
        self.current_action_index = 0 if len(self.actions) > 0 else None

        #link actions in action list to themselves
        for act in self.actions:
            act.actions = [next((self_act for self_act in self.actions if
                    self_act.id == child_act.id), None)
                        if child_act.type == Action.ACTION_QUEUE else child_act
                        for child_act in
                        act.actions]

        self._state_publisher = rospy.Publisher('experiment_state',
                                                ExperimentState)
        self._state_service = rospy.Service('get_experiment_state', GetExperimentState,
                      self.get_experiment_state_cb)

        self._update_experiment_state()

    def _selected_step_cb(self, selected_step):
        '''Updates the selected step when interactive
        markers are clicked on'''
        self._selected_step = selected_step
        self._update_experiment_state()

    def get_experiment_state_cb(self, dummy):
        ''' Response to the experiment state service call'''
        return GetExperimentStateResponse(self._get_experiment_state())

    def _update_experiment_state(self):
        ''' Publishes a message with the latest state'''
        state = self._get_experiment_state()
        self._state_publisher.publish(state)

    def _get_experiment_state(self):
        ''' Creates a message with the latest state'''
        return ExperimentState(
            (self.actions[self.current_action_index].to_string()
            if self.current_action_index != None
            else ""),
            map(lambda act: act.name, self.actions),
            map(lambda act: act.id, self.actions),
            0 if self.current_action_index == None else self.current_action_index,
            self._selected_step,
            self.object_names)

    def _get_ref_frames(self, arm_index):
        ''' Returns the reference frames for the steps of the
        current action in array format'''
        ref_frames = []
        for i in range(self.n_frames()):
            action = self.actions[self.current_action_index]
            ref_frame = action.get_step_ref_frame(arm_index, i)
            ref_frames.append(ref_frame)
        return ref_frames

    def select_action_step(self, step_id):
        ''' Makes the interactive marker for the indicated action
        step selected, by showing the 6D controls'''
        #self.actions[self.current_action_index].select_step(step_id)
        self._selected_step = step_id

    def _get_participant_id(self):
        '''Gets the experiment number from the command line'''
        while (self._exp_number == None):
            try:
                self._exp_number = int(raw_input(
                                    'Please enter participant ID:'))
            except ValueError:
                rospy.logerr("Participant ID needs to be a number")

            self._data_dir = Session._get_data_dir(self._exp_number)
            if (not os.path.exists(self._data_dir)):
                os.mkdir(self._data_dir)
            else:
                rospy.logwarn('A directory for this participant ' +
                              'ID already exists: ' + self._data_dir)
                overwrite = raw_input('Do you want to overwrite? ' +
                                      'Type r to reload the last state ' +
                                      'of the experiment. [y/n/r]')
                if (overwrite == 'y'):
                    continue
                elif (overwrite == 'n'):
                    self._exp_number = None
                elif (overwrite == 'r'):
                    self._is_reload = True
                else:
                    rospy.logerr('Invalid response, try again.')

    @staticmethod
    def _get_data_dir(exp_number):
        '''Returns the directory where action information is saved'''
        return (rospy.get_param('~dataRoot') +
                    '/data/experiment' + str(exp_number) + '/')

    def save_session_state(self, is_save_actions=True):
        '''Saves the session state onto hard drive'''
        # exp_state = dict()
        # exp_state['nProgrammedActions'] = self.n_actions()
        # exp_state['currentProgrammedActionIndex'] = self.current_action_index
        # state_file = open(self._data_dir + 'experimentState.yaml', 'w')
        # state_file.write(yaml.dump(exp_state))
        # state_file.close()

        if (is_save_actions):
            for i in range(self.n_actions()):
                self.actions[i].save()

    def new_action(self):
        '''Creates new action'''
        self.current_action_index = len(self.actions)
        self._selected_step = 0
        newAct = Action(act_type=Action.ACTION_QUEUE, act_name="Unnamed")
        newAct.actions = []
        newAct.save()
        self.actions.append(newAct)
        self._update_experiment_state()

    def n_actions(self):
        '''Returns the number of actions programmed so far'''
        return len(self.actions)

    def get_current_action(self):
        '''Returns the current action'''
        return self.actions[self.current_action_index]

    def clear_current_action(self):
        '''Removes all steps in the current action'''
        if (len(self.actions) > 0):
            self.actions[self.current_action_index].actions = []
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def undo_clear(self):
        '''Undo the effect of clear'''
        rospy.logwarn('Unimplemented undo')

    def save_current_action(self):
        '''Save current action onto hard drive'''
        if (self.current_action_index != None):
            self.actions[self.current_action_index].save()
            self.save_session_state(is_save_actions=False)
            return True
        else:
            rospy.logwarn('No skills created yet.')
            return False

    def add_step_to_action(self, step_act):
        '''Add a new pose step to the current action'''
        if (self.current_action_index != None):
            self.actions[self.current_action_index].actions.insert(self._selected_step, step_act)
            self._selected_step += 1
            rospy.loginfo("added a new step at " + str(self._selected_step))
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def add_action_step_action(self, act_name):
        '''add a new step to the current action, where the step is another action'''
        act = next((act for act in self.actions
                if act.name == act_name), None)
        if (act != None):
            self.actions[self.current_action_index].actions.insert(self._selected_step, act)
            self._update_experiment_state()
            return True
        else:
            rospy.logwarn("Action " + act_name + " not found")
            return False

    def delete_last_step(self):
        '''Removes the previous selected step of the action'''
        if (self.current_action_index != None):
            if (self._selected_step > 0):
                self._selected_step -= 1
                rospy.loginfo("removing step " + str(self._selected_step))
                self.actions[self.current_action_index].actions.pop(self._selected_step)
            else:
                rospy.logwarn('Trying to delete step when selected index is zero')
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def resume_deleted_step(self):
        '''Resumes the deleted step'''
        rospy.logwarn('Unimplemented undo')

    def repeat_step(self):
        '''copies previous step'''
        if (self.current_action_index != None):
            if (self._selected_step > 0):
                self.actions[self.current_action_index].actions.insert(self._selected_step,
                        self.actions[self.current_action_index].actions[self._selected_step - 1].copy())
                self._selected_step += 1
            else:
                rospy.logwarn('Trying to repeal step when selected index is zero')
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def switch_to_action(self, action_number):
        '''Switches to indicated action'''
        '''TODO: switch action_number to action_id'''
        if (action_number < len(self.actions) and action_number >= 0):
            self.current_action_index = action_number
            self._selected_step = len(self.actions[self.current_action_index].actions)
            self._update_experiment_state()
            return True
        else:
            rospy.logwarn('Cannot switch to action '
                            + str(action_number))
            return False

    def next_action(self):
        '''Switches to next action'''
        return self.switch_to_action(self.current_action_index + 1)

    def previous_action(self):
        '''Switches to previous action'''
        return self.switch_to_action(self.current_action_index - 1)

    def switch_to_action_by_name(self, action_name):
        '''swithc to another action by the action name'''
        return self.switch_to_action(next((i for i, act in enumerate(self.actions)
                if act.name == action_name), -1))

    def name_action(self, new_name):
        '''name the action'''
        if (len(self.actions) > 0):
            self.actions[self.current_action_index].name = new_name
            self._update_experiment_state()

    def set_action_landmark_type(self, descriptor, arm_ind, prev_mark, new_mark):
        '''Sets the action landmark type'''
        if (len(self.actions) > 0):
            cur_act = self.actions[self.current_action_index].actions[
                self._selected_step]
            cur_act.landmark_types[arm_ind] = descriptor
            pose = cur_act.arms[arm_ind]
            pose.pose.position.x = pose.pose.position.x - new_mark.pose.position.x + prev_mark.pose.position.x
            pose.pose.position.y = pose.pose.position.y - new_mark.pose.position.y + prev_mark.pose.position.y
            pose.pose.position.z = pose.pose.position.z - new_mark.pose.position.z + prev_mark.pose.position.z
            self._update_experiment_state()

    def set_step_scan(self, scan_code):
        if (len(self.actions) > 0):
            self.actions[self.current_action_index].actions[self._selected_step].scan_code = scan_code
            self._update_experiment_state()

    def n_frames(self):
        '''Returns the number of frames'''
        if (self.current_action_index != None):
            return len(self.actions[self.current_action_index].actions)
        else:
            rospy.logwarn('No skills created yet.')
            return 0

    def frame_types(self):
        '''returns frame types'''
        if (self.current_action_index != None):
            return map(lambda act: act.type, self.actions[self.current_action_index].actions)
        else:
            rospy.logwarn('No skills created yet.')
            return []

    def update_object_names(self, new_names):
        self.object_names = new_names
