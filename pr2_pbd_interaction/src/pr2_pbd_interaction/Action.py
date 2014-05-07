'''Representation of a programmed action.

This module handles the interface between the YAML and Python
represetnations of a programmed action.
'''

# These ROS imports must come first.
import roslib
roslib.load_manifest('pr2_pbd_interaction')

# Standard library imports.
import functools
import os
import yaml

# 3rd party imports (e.g. ROS).
import rospy

class Action:
    '''Pthon representation of a programmed action.

    An instance of this class represents one programmed action.

    Actions can have one of the following types:
        Action.ACTION_QUEUE
        Action.POSE
        Action.GRIPPER
        Action.TRAJECTORY
    '''

    action_directory = rospy.get_param("/pr2_pbd_interaction/actionsRoot")
    file_extension = ".yaml"

    # The list of Action types.
    # TODO(max): There are a couple options for how we could clean this
    # up. (1) Use Enums (introduced in Python 3.4, could import them).
    # (2) Do an OO-approach by making a base class and having each of
    # these extend it. These look like step types rather than action
    # types, though, and there's no clear reason now, so I'll wait.
    ACTION_QUEUE = 0
    POSE = 1
    GRIPPER = 2
    TRAJECTORY = 3

    ####################################################################
    # INSTANCE METHODS
    ####################################################################

    def __init__(self, act_id=None, act_type=None, act_name=None):
        '''Create an action without initialization.

        Args:
            act_id (int, optional): id of the action. Defaults to None.
            act_type (int [see Action.* class variables], optional):
                Type of the action. Defaults to None.
            act_name (str, optional): Name of the action. Defaults to
                None.
        '''
        self.id = act_id
        self.type = act_type
        self.name = act_name

    def save(self):
        '''Saves action to file with its id.

        Finds a new id if it doesn't have one yet, avoiding collisions
        with existing files. Saves to Action.action_directory.
        '''
        Action.ensure_dir_exists()
        if (self.id is None):
            self.id = 0
            while (os.path.isfile(self.get_file(self.id))):
                self.id += 1
        act_file = open(self.get_file(self.id), 'w')
        act_file.write(self.to_string())
        act_file.close()

    def to_string(self):
        '''Serializes action into YAML string.

        Returns:
            str: YAML representation of this action.
        '''
        return yaml.dump(self)

    def copy(self):
        '''Returns a shallow copy of the action.

        Copies sub-steps but not sub-actions.

        Returns:
            Action : new action that is shallow copy of self.
        '''
        return Action.from_string(self.to_string())

    ####################################################################
    # CLASS METHODS
    ####################################################################

    @classmethod
    def ensure_dir_exists(cls):
        '''Ensures that the default actiondirectory exists by first
        checking and then creating it if it doesn't.
        '''
        if not os.path.exists(cls.action_directory):
            os.makedirs(cls.action_directory)

    ####################################################################
    # STATIC METHODS
    ####################################################################

    @staticmethod
    def get_file(action_id):
        '''Get a specific action file from an action id.

        Args:
            action_id (int): The id of the action to get.

        Returns:
            str: The full path to the action file specified by
                action_id.
        '''
        return Action.action_directory + str(action_id) + Action.file_extension

    @staticmethod
    def get_saved_actions():
        '''Returns a list of all Actions in the default actions dir.

        Returns:
            list(Action)
        '''
        Action.ensure_dir_exists()
        candidates = [os.path.join(Action.action_directory, f) for f in
                os.listdir(Action.action_directory)]
        return [Action.load(c) for c in candidates if os.path.isfile(c)
                and c.endsWith(Action.file_extension)]

    @staticmethod
    def load(act_f_id):
        '''Load an action from a file.

        Args:
            act_f_id (int|str): Either the action id or the full path of
                the action to load.

        Returns:
            Action
        '''
        Action.ensure_dir_exists()
        if type(act_f_id) is int:
            file_path = Action.get_file(act_f_id)
        else:
            file_path = act_f_id
        # TODO(max): How is this used? We should either document the
        # exception so callers can try/except it, or return None (and
        # document it).
        with open(file_path, 'r') as act_file:
            return Action.from_string(act_file)

    @staticmethod
    def from_string(act_str):
        '''Load an action from a YAML-formatted string.

        Args:
            act_str (str): YAML-representation of action in a string, as
                retruned by to_string().

        Returns:
            Action : An Action instance with all state from act_str.
        '''
        return yaml.load(act_str)
