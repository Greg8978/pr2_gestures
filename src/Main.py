#!/usr/bin/env python

import BML_reader

import rospy

from Config import *

class Main:
    '''
    starting point of the routine by reading a BML file
    TODO: add an interface to load BML files and pause the threads.
    '''
    
    def __init__(self):
        if(DEBUG_MODE): print("init Main")
        self.my_reader = BML_reader.BML_reader()
        self.BML_string = ''
        
    def main(self):
        
       more = True
       while(more):
            var = raw_input("0: end , 1: live, 2: macro : ")
            if (var == '0'):
                more = False
            elif(var == '2'):
                var = raw_input("Enter macro name: ")
                bml_file = './src/data/macro/BML_' + var + '.xml'
                self.my_reader.set_dom_from_file(bml_file, ("",""),(("r_shoulder_pan_joint",0.0),("l_shoulder_pan_joint",0.0),("head_pan_joint",0.0)))
                self.my_reader.handleBML(self.my_reader.get_dom())
            elif(var == '1'):
                self._create_BML_live()
                self.my_reader.set_dom_from_string(self.BML_string,("",""),(("r_shoulder_pan_joint",0.0),("l_shoulder_pan_joint",0.0),("head_pan_joint",0.0)))
                self.my_reader.handleBML(self.my_reader.get_dom())           
            else:
                print("wrong option") 
                
       rospy.signal_shutdown("Quitting")
 
        
        
    def _create_BML_live(self):
        bml = True
        self.BML_string = self.BML_string + '<bml>'
        
        while(bml):
            var = raw_input("(p: print), 0: END everything , 1: New block of actions : ")
            if (var == '0'):
                self.BML_string = self.BML_string + '</bml>'
                bml = False
            elif(var == '1'):
                self.BML_string = self.BML_string + '<synchronize>'
                self._add_sync()
                self.BML_string = self.BML_string + '</synchronize>'
            elif(var == 'p'):
                print "BML: ", self.BML_string
            else:
                print("wrong option")
        

    def _add_sync(self):
        more_actions = True
        while(more_actions):
            var = raw_input("(p: print), 0: END this block of actions , 1: New speech, 2: New behavior : ")
            if (var == '0'):
                more_actions = False
            elif(var == '1'):
                self._add_speech()
            elif(var == '2'):
                self._add_behavior()
            elif(var == 'p'):
                print "BML: ", self.BML_string
            else:
                print("wrong option")
    
    def _add_behavior(self):
        offset = raw_input(" enter OFFSET time in seconds :")
        param = raw_input(" enter PARAM, angle in degrees:")
        behavior = raw_input(" enter behavior name : ")
        self.BML_string = self.BML_string + '<behavior param="' + param +'" offset="' + offset + '">' + behavior + '</behavior>'
    
    def _add_speech(self):
        offset = raw_input(" enter offset time in seconds :")
        speech = raw_input(" enter speech name : ")
        self.BML_string = self.BML_string + '<speech offset="' + offset + '" type="mp3">' + speech + '</speech>'
        
        
if __name__ == '__main__': 
    Main().main()
        
