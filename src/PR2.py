'''
Created on Sep. 29, 2014
Last change on Oct. 20, 2014

@author: Javier
'''
from Config import *
import math

class PR2:
    '''
    @Class: Describes the PR2 and how to move it. 
               
    The following attributes are LIST:
    @attribute self._joints_l: names of the left joints
    @attribute self._states_l: states of the left joints
    @attribute self._joints_r: names of the right joints
    @attribute self._states_r: states of the right joints
    @attribute self._joints_head: names of the head joints
    @attribute self._states_head: states of the head joints
    @attribute self._speed: time to reach each state
    '''
    def __init__(self):
        if(DEBUG_MODE): print("init PR2")
        self._joints_l = []
        self._states_l = []
        self._joints_r = []
        self._states_r = []
        self._joints_head = []
        self._states_head = []
        self._speed = []
        
    '''
    @method set_PR2: Set the PR2 from the description
    @param description: from where the information is collected the set the attributes
    @note: description is a Behavior_description
    '''
    def set_PR2(self,description, params):
        self._states_l = []
        self._states_r = []
        self._states_head = []     
        self._speed = description.get_speed()
        
        joints = description.get_joint_names()
        states = description.get_joint_states()
        states_trans = zip(*states)
        for state , joint in zip(states_trans,joints):
            
            ''' the param is added to 3 joints'''
            for param in params:
                state = self._add_param_to_PR2(param[0], joint,state, param[1])
            
            if(joint[:1] == 'l'):
                self._joints_l.append(joint)
                self._states_l.append(state)
            elif(joint[:1] == 'r'):
                self._joints_r.append(joint)
                self._states_r.append(state)
            elif(joint[:1] == 'h'):
                self._joints_head.append(joint)
                self._states_head.append(state)
            else:
                print('ERROR PR2 joint not recognized : %s' % joint)
    
        self._states_l = zip(*self._states_l)
        self._states_r = zip(*self._states_r)
        self._states_head = zip(*self._states_head)

    '''
    @method _add_param_to_PR2: add value to a specific joint.
    @param joint_name: Joint name that will see its states modified
    @param current_joint: joint (joints that will be check to see if it match with the previous one)
    @param current_state: list of the states that are currently process.
    @param param: which will be added to the current state.
    
    @return: the list of states modified or not depending if its the right joint.
    '''
    def _add_param_to_PR2(self,joint_name, current_joint,current_state, param):

        if(joint_name == current_joint):
            param_in_rad = math.radians(param)
            if(DEBUG_MODE): print(param_in_rad)
            
            res = [(x + param_in_rad) for x in current_state]
        else:
            res = current_state

        return res
    
    '''
    @method get_r_joints and get_r_states: Getter for RIGHT joints names and states
    @return self._joints_r or self._states_r
    ''' 
    def get_r_joints(self):
        return self._joints_r
    def get_r_states(self):
        return self._states_r    
    '''
    @method get_l_joints and get_l_states: Getter for LEFT joints names and states
    @return self._joints_l or self._states_l
    ''' 
    def get_l_joints(self):
        return self._joints_l
    def get_l_states(self):
        return self._states_l
    '''
    @method get_head_joints and get_head_states: Getter for HEAD joints names and states
    @return self._joints_head or self._states_head
    ''' 
    def get_head_joints(self):
        return self._joints_head
    def get_head_states(self):
        return self._states_head
    '''
    @method get_speed: Getter for the speed (between the joints)
    @return self._speed
    ''' 
    def get_speed(self):
        return self._speed   
    
    '''
    @method _string_to_float: String to float method
    
    @note: if string is empty i.e. if we don't specify it, we set it by default to 0 
    
    @return string: string that will be transformed in float
    '''      
    def _string_to_float(self, string):
        if(string == ""): string = "0"
        return float(string)
    
        
    '''
    @method print_test_descrip: Print all the attributes of this class.
    @todo: to remove.
    ''' 
    def print_test_descrip(self):
        print("left : %s" % self._joints_l)
        for state in self._states_l:
            print(state)
        print("right : %s" % self._joints_r)
        for state in self._states_r:
            print(state)
        print("head : %s" % self._joints_head)
        for state in self._states_head:
            print(state)
        print("speed : %s" % self._speed) 