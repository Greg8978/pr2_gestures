'''
Created on 21 juil. 2014
Last change on Oct. 22, 2014

@author: Javier
'''

import Behavior_reader
import PR2
import Behavior_ros


import roslib
import signal
roslib.load_manifest('pr2_tuck_arms_action')

import rospy

import os
import sys
import time
import math

from trajectory_msgs.msg import *
from actionlib_msgs.msg import *
from pr2_controllers_msgs.msg import *
from pr2_common_action_msgs.msg import *
import getopt
import actionlib

import datetime

from Config import *

class Behavior:
    '''
    @Class: Manage the execution of the behaviors.
               
    @attribute self.my_behavior_reader: Behavior_reader where is manage the reading and gathering the behavior information.
    @attribute self.my_PR2: PR2 class which is given as parameter to Behavior_ros once it is set.
    @attribute self.action_name: ROS node name ??? 
    @attribute self.tuck_arms_action_server: what will connect this module to the Robot via ros topics
    
    For a new Robot:
    @todo: self.my_PR2 we might want change this attribute for other robots than the PR2.
    @todo: create the new_robot class
    '''
    def __init__(self):
        if(DEBUG_MODE): print("init Behavior" )
        self.my_behavior_reader = Behavior_reader.Behavior_reader()
        
        '''Depends on the robot'''
        self.my_PR2 = PR2.PR2()

        self.action_name = 'tuck_armst' + datetime.datetime.now().strftime('%S')

        if(DEBUG_MODE): print(self.action_name)
        rospy.init_node(self.action_name)
        
        rospy.sleep(0.001)  # wait for time
        self.tuck_arms_action_server = Behavior_ros.Behavior_ros(self.action_name,self.my_PR2)

    '''
    @method execute_behavior: Execute a behavior in 3 steps:
                                        - Read the behavior description form a file
                                        - Grab the information previously read
                                        - Perform the behavior the previous information
    @param behavior: name of the behavior to execute
    @param param: which will be added to some joints to follow a human
    ''' 
    def execute_behavior(self, behavior,param_behavior):
        self.my_behavior_reader.read_behavior_from_file(behavior)
        
        to_exec = self.my_behavior_reader.get_behavior_description()
        
        self._select_and_execute(to_exec,param_behavior)
        if(PRINT_BML): print("    <behavior>%s</behavior>" % (behavior))
            
    '''
    @method _select_and_execute: Execute by sending the goal to the action server
    @param behavior_description_to_exec: It's a Behavior_description which contains all the information to perform the behavior
    @todo: Changes to do here for a new robot (we are working here with the PR2)
    '''
    def _select_and_execute(self,behavior_description_to_exec, param_behavior):
        self.my_PR2.set_PR2(behavior_description_to_exec, param_behavior)
        
        goal = TuckArmsGoal()
        tuck_arm_client = actionlib.SimpleActionClient(self.action_name, TuckArmsAction)
        if(PRINT_ROS): rospy.logdebug('Waiting for action server to start')
        tuck_arm_client.wait_for_server(rospy.Duration(200.0))
        if(PRINT_ROS): rospy.logdebug('Sending goal to action server')
        tuck_arm_client.send_goal_and_wait(goal)
        if(PRINT_ROS): rospy.loginfo(' Finish ')
        
# useless to remove
#        quit_when_finished = True
#         if quit_when_finished:
#             rospy.loginfo(' Quit ')
#             rospy.signal_shutdown("Quitting")
#         
#         rospy.spin()

        

            
        
        
        
        
    