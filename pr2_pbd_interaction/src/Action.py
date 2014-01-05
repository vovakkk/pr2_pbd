import roslib
roslib.load_manifest('pr2_pbd_interaction')

from ObjectType import ObjectType

#functional code
from functools import partial

import os.path
from os import listdir
from os.path import isfile, join
from xml.etree.ElementTree import ElementTree, TreeBuilder, tostring as elementtostring
from xml.etree.ElementTree import fromstring as elementfromstring
import rospy

class Action:
    ACTION_DIRECTORY = "/home/vladimir/pbd_actions/"
    FILE_EXTENSION = ".xml"

    ACTION_QUEUE = 0
    POSE = 1
    GRIPPER = 2
    TRAJECTORY = 3
    ARM_PROPS = { "position" : ["x", "y", "z"], "orientation": ["x", "y", "z", "w" ]}
    
    def get_file(self, action):
        return Action.ACTION_DIRECTORY + str(action) + Action.FILE_EXTENSION
    
    @staticmethod
    def get_saved_actions():
        return map(Action, 
            filter(lambda f: f.endswith(Action.FILE_EXTENSION),
                filter(isfile, 
                    map(partial(join, Action.ACTION_DIRECTORY), 
                        listdir(Action.ACTION_DIRECTORY)))))
    
    def __init__(self, file=None, id=None):
        self.type = 0
        self.name = ''
        if (id != None):
            file = self.get_file(id)
        self.id = id
        if ((file != None) and (os.path.isfile(file))):
            tree = ElementTree()
            tree.parse(file)
            root = tree.getroot()
            self._read_action(root, self)
    
    def _read_action(self, act_el, act_obj):
        '''records data from the action xml element act_el into act_obj, returns act_obj'''
        act_obj.type = int(act_el.get("type"))
        if ("id" in act_el.attrib):
            act_obj.id = int(act_el.attrib["id"])
        act_obj.name = act_el.find("name").text
        if (act_obj.name == None):
            act_obj.name = ""
        
        def read_arm(el):
            pose = {}
            for prop in Action.ARM_PROPS:
                pose[prop] = {}
                for prop2 in Action.ARM_PROPS[prop]:
                    pose[prop][prop2] = float(
                            el.find(prop).find(str(prop2)).text)
            pose["joints"] = map(lambda ar_el: float(ar_el.text), list(el.find("joints")))
            return pose
        
        def read_arms(el):
            return map(lambda a_ind: 
                    read_arm(el.find("arms/arm[@index='" + str(a_ind) + "']")), [0, 1])
        
        if (act_obj.type == Action.ACTION_QUEUE):
            act_obj.actions = map(lambda el: 
                    self._read_action(el, Action()) if el.get("inline") == "True"
                    else Action(int(el.get("id"))), list(act_el.find("actions")))
        elif (act_obj.type == Action.POSE):
            act_obj.arms = read_arms(act_el.find("pose"))
            act_obj.target = ObjectType(int(act_el.find("target/type_id").text))
        elif (act_obj.type == Action.GRIPPER):
            act_obj.is_open = bool(act_el.find("gripper").find("is_open").text)
            act_obj.arm_index = int(act_el.find("gripper").find("arm_index").text)
        elif (act_obj.type == Action.TRAJECTORY):
            act_obj.poses = map(lambda pose_el: {
                    "arms" : read_arms(pose_el),
                    "timing" : int(pose_el.find("timing").text)
                }, list(act_el.find("poses")))
        return act_obj
    
    def _write_action(self, builder, act_obj):
        '''writes action from act_obj to builder'''
        builder.start("name", {})
        builder.data(act_obj.name)
        builder.end("name")
        
        def write_arms(arms):
            builder.start("arms", {})
            for a_ind in [0, 1]:
                builder.start("arm", {"index" : str(a_ind)})
                for prop in Action.ARM_PROPS:
                    builder.start(prop, {})
                    for prop2 in Action.ARM_PROPS[prop]:
                        builder.start(str(prop2), {})
                        builder.data(str(arms[a_ind][prop][prop2]))
                        builder.end(str(prop2))
                    builder.end(prop)
                builder.start("joints", {})
                for joint in arms[a_ind]["joints"]:
                    builder.start("i", {})
                    builder.data(str(joint))
                    builder.end("i")
                builder.end("joints")
                builder.end("arm")
                    
            builder.end("arms")
        
        if (act_obj.type == Action.ACTION_QUEUE):
            builder.start("actions", {})
            for action in act_obj.actions:
                if (action.id == None):
                    builder.start("action", { "type" : str(action.type), "inline" : "True" })
                    self._write_action(builder, action)
                    builder.end("action")
                else:
                    builder.start("action", { "inline" : "False", "id" : str(action.id) })
                    builder.end("action")
            builder.end("actions")
        elif (act_obj.type == Action.POSE):
            builder.start("pose", {})
            write_arms(act_obj.arms)
            builder.end("pose")
            builder.start("target", {})
            builder.start("type_id", {})
            builder.data(str(act_obj.target.type_id))
            builder.end("type_id")
            builder.end("target")
        elif (act_obj.type == Action.GRIPPER):
            builder.start("gripper", {})
            builder.start("is_open", {})
            builder.data(str(act_obj.is_open))
            builder.end("is_open")
            builder.start("arm_index", {})
            builder.data(str(act_obj.arm_index))
            builder.end("arm_index")
            builder.end("gripper")
        elif (act_obj.type == Action.TRAJECTORY):
            builder.start("poses", {})
            for pose in act_obj.poses:
                builder.start("pose", {})
                write_arms(pose["arms"])
                builder.start("timing", {})
                builder.data(str(pose["timing"]))
                builder.end("timing")
                builder.end("pose")
            builder.end("poses")        
    
    def save(self):
        '''saves action to file'''
        if (self.id == None):
            self.id = 0
            while (os.path.isfile(self.get_file(self.id))):
                self.id += 1
        builder = TreeBuilder()
        builder.start("action", { "id" : str(self.id), "type" : str(self.type),
                                  "inline" : "True" })
        self._write_action(builder, self)
        builder.end("action")
        doc = ElementTree(builder.close())
        doc.write(self.get_file(self.id))
    
    def write_in(self, builder):
        '''writes xml for this action including action element'''
        builder.start("action", { "type" : str(self.type), "inline" : "True" });
        self._write_action(builder);
        builder.end("action");
        
    def to_string(self):
        '''gets the full xml representing this action'''
        builder = TreeBuilder()
        builder.start("action", { "id" : str(self.id), "type" : str(self.type),
                                  "inline" : "True" })
        self._write_action(builder, self)
        builder.end("action")
        return elementtostring(builder.close(), "utf-8")
    
    def from_string(self, xml_str):
        tree = ElementTree(elementfromstring(xml_str))
        root = tree.getroot()
        self._read_action(root, self)
        
    def copy(self):
        copy = Action()
        copy.from_string(self.to_string())
        return copy