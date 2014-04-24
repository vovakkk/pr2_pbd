import roslib
roslib.load_manifest('pr2_pbd_interaction')

from ObjectType import ObjectType

#functional code
from functools import partial

import os.path
from os import listdir
from os.path import isfile, join
import rospy
import yaml

class Action:
    ACTION_DIRECTORY = rospy.get_param("/pr2_pbd_interaction/actionsRoot")
    FILE_EXTENSION = ".yaml"

    ACTION_QUEUE = 0
    POSE = 1
    GRIPPER = 2
    TRAJECTORY = 3

    '''get a specifi action file from action id'''
    @staticmethod
    def get_file(action):
        return Action.ACTION_DIRECTORY + str(action) + Action.FILE_EXTENSION

    @staticmethod
    def check_dir_exists():
        if not os.path.exists(Action.ACTION_DIRECTORY):
            os.makedirs(Action.ACTION_DIRECTORY)
    
    '''get list of saved action located in the actions folder'''
    @staticmethod
    def get_saved_actions():
        Action.check_dir_exists()
        return map(Action.load, 
            filter(lambda f: f.endswith(Action.FILE_EXTENSION),
                filter(isfile, 
                    map(partial(join, Action.ACTION_DIRECTORY), 
                        listdir(Action.ACTION_DIRECTORY)))))
    
    '''load an action
        act_f_id is an action id (int) or action file path'''
    @staticmethod
    def load(act_f_id):
        Action.check_dir_exists()
        file_path = ""
        if (type(act_f_id) is int):
            file_path = Action.get_file(act_f_id)
        else:
            file_path = act_f_id
        act_file = open(file_path, 'r')
        act = Action.from_string(act_file)
        act_file.close()
        return act

    '''load an action from a string'''
    @staticmethod
    def from_string(str):
        return yaml.load(str)

    '''initialize an actions
        id - action id
        type - action type
        name - action name'''
    def __init__(self, id=None, type=None, name=None):
        self.type = type
        self.name = name
        self.id = id     
    
    '''save action to file'''
    def save(self):
        Action.check_dir_exists()
        '''saves action to file'''
        if (self.id == None):
            self.id = 0
            while (os.path.isfile(self.get_file(self.id))):
                self.id += 1
        act_file = open(self.get_file(self.id), 'w')
        act_file.write(self.to_string())
        act_file.close()
    
    '''convert action to a string'''
    def to_string(self):
        '''gets the yaml representing this action'''
        return yaml.dump(self)
        
    '''make a shallow copy of the action (sub actions are not copied, sub steps are)'''
    def copy(self):
        return Action.from_string(self.to_string())