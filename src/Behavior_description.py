'''
Created on Aug. 29, 2014
Last change on Oct. 20, 2014

@author: Javier
'''
import copy

from Config import *

class Behavior_description:
    '''
    @Class: Concentrate the generic description of a behavior.
    
    The following attributes are LIST:
    @attribute self._joint_names: names of the joints to move
    @attribute self._joint_states: states of joints 
    @attribute self._speed: time to reach each state
    '''
    def __init__(self):
        if(DEBUG_MODE): print("init Behavior_description" )
        self._joint_names = []
        self._joint_states = []
        self._speeds = []
    
    '''
    @method set_joint_states: Transpose the matrix
    @param joint_states: Matrix (list of list)
    '''
    def set_joint_states(self, joint_states):
        self._joint_states = copy.copy(zip(*joint_states))

    '''
    @todo: to remove.
    @method print_test_descrip: Print all the attributes of this class.
    @note: Speed / Joint Names / Joint States
    '''  
    def print_test_descrip(self):
        print("    %s" % self._speeds)
        print("%s" % self._joint_names)
        for state in self._joint_states:
            print(state)
            
    '''
    @method set_speed: Setter for the speed (time between the joint states)
    @return set the attribute speeds (list of speed) 
    '''
    def set_speed(self, speeds):
        self._speeds = speeds
        
    '''
    @method get_speed: Getter for SPEED list
    @return self._speed list of speeds
    '''  
    def get_speed(self):
        return self._speeds
    
    '''
    @method add_joint_mane: Add a new joint name to self._joint_names (list of joint names)
    @note: use in Behavior_reader class in the method read_behavior_from_file
    @todo: check this list //////////!\\\\\\\\\\\ might have a bug here (not critic)
    '''  
    def add_joint_mane(self,joint_name):    
        self._joint_names.append(joint_name) 
        
    '''
    @method get_joint_names: Getter of self._joint_names
    @return self._joint_names list of the current joint names
    '''  
    def get_joint_names(self):
        return self._joint_names
    
    '''
    @method get_joint_states: Getter of self._joint_states
    @return self._joint_states list of the current joint states
    '''  
    def get_joint_states(self):
        return self._joint_states
    