'''
Created on Aug. 25, 2014
Last change on Oct. 20, 2014

@author: Javier
'''

import Behavior_description

from Config import *

class Behavior_reader:
    '''
    @Class: Read the wanted description from a specific file and stored in a Behavior_description object.

    @attribute self.my_Behavior_description: Behavior_description where we store the name/state joints and speed.
    '''
    def __init__(self):
        if(DEBUG_MODE): print("init Behavior_reader" )
        self.my_Behavior_description = Behavior_description.Behavior_description()

    '''
    @method read_behavior_from_file: Read a file and set the "description behavior" parameters
    @param name_file: from where the information is collected to execute the behavior
    @note: Used in Behavior class when a behavior needs to be executed and we need the information how to perform it.
    @note:     Read line by line : Ex: jointName state1 state2 ..... stateN
    @note:     Last line (position not mandatory) speed sp1 sp2 ... spN
    '''   
    def read_behavior_from_file(self, name_file):
        full_path = PATH + name_file + '.txt'
        
        list_of_states_to_transpose = []
        
        with open(full_path, 'r') as courent_file:
            #read each line, one after the other
            content_line = courent_file.readlines()
            for line in content_line:
                words = line.split()
                if words[0] == 'speed':
                    # store speed
                    del words[0]
                    self._list_to_float(words)
                    self.my_Behavior_description.set_speed(words)
                else:
                    # store joint names
                    self.my_Behavior_description.add_joint_mane(words[0])
                    del words[0]
                    # store joint states
                    self._list_to_float(words)   
                    list_of_states_to_transpose.append(words) 
                    
        self.my_Behavior_description.set_joint_states(list_of_states_to_transpose)

        courent_file.close()    

    '''
    @method get_behavior_description: Getter for the setted description
    @return: self.my_Behavior_description
    @note: Called in Behavior class once the behavior is set to be able to executed a behavior
    '''
    def get_behavior_description(self):
        return self.my_Behavior_description

    '''
    @method _list_to_float: Set all the elements of the list to floats (String to float)
    @param list_to_trans: list of strings to change to floats
    @note: Called in function "read_behavior_from_file" when reading strings from the file.
    '''
    def _list_to_float(self, list_to_trans):
        courent_val_index = 0
        for val in list_to_trans:    
            list_to_trans[courent_val_index] = float(val)
            courent_val_index +=1