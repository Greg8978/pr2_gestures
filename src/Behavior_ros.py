'''
Created on Aug. 29, 2014
Last change on Oct. 24, 2014

@author: Javier
'''

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

import PR2

from Config import *

class Behavior_ros:
    '''
    @Class: Make the connection through ROS between this module and the Robot (working on PR2 with morse simulation)
    
    @attribute self._pr2: is here to be able to match the position of the states of the PR2
    
    @attribute self.r_received: True if the right side of the robot has received a new state
    @attribute self.l_received: True if the left side of the robot has received a new state
    @attribute self.head_received: True if the head of the robot has received a new state
    
    @attribute self.node_name: Name of the ROS node
    
    @attribute self.success: True if all went right (always true for the moment)
    
    @attribute self.move_duration: define the time to go to the next state (in seconds)
    
    @attribute self.left_joint_client: connection to the action client, though what we send the left goals 
    @attribute self.right_joint_client: connection to the action client, though what we send the right goals 
    @attribute self.head_joint_client: connection to the action client, though what we send the head goals 
    
    @attribute self.action_server: connection to the action server
    '''
    def __init__(self,node_name, pr2):
        self._pr2 = pr2
        
        self.r_received = False
        self.l_received = False
        self.head_received = False 
  
        self.node_name = node_name
    
        self.success = True
    
        # Get controller name and start joint trajectory action clients
        self.move_duration = rospy.get_param('~move_duration', 2.5)
        r_action_name = rospy.get_param('~r_joint_trajectory_action', 'r_arm_controller/joint_trajectory_action')
        l_action_name = rospy.get_param('~l_joint_trajectory_action', 'l_arm_controller/joint_trajectory_action')
        head_action_name = rospy.get_param('~head_joint_trajectory_action', 'head_controller/joint_trajectory_action')
        
        self.left_joint_client = client = actionlib.SimpleActionClient(l_action_name, JointTrajectoryAction)
        self.right_joint_client = client = actionlib.SimpleActionClient(r_action_name, JointTrajectoryAction)
        self.head_joint_client = client = actionlib.SimpleActionClient(head_action_name, JointTrajectoryAction)
    
        # Connect to controller state
        rospy.Subscriber('l_arm_controller/state', JointTrajectoryControllerState ,self._state_callback)
        rospy.Subscriber('r_arm_controller/state', JointTrajectoryControllerState ,self._state_callback)
        rospy.Subscriber('head_controller/state', JointTrajectoryControllerState ,self._state_callback)
    
        # Wait for joint clients to connect with timeout
        if not self.left_joint_client.wait_for_server(rospy.Duration(30)):
            rospy.logerr("pr2_tuck_arms: left_joint_client action server did not come up within timelimit")
        if not self.right_joint_client.wait_for_server(rospy.Duration(30)):
            rospy.logerr("pr2_tuck_arms: right_joint_client action server did not come up within timelimit")
        if not self.head_joint_client.wait_for_server(rospy.Duration(30)):
            rospy.logerr("pr2_tuck_head: head_joint_client action server did not come up within timelimit")
            
        # Construct action server
        self.action_server = actionlib.simple_action_server.SimpleActionServer(node_name,TuckArmsAction, self._execute_callback)

    '''
    @method _state_callback: call back function to set when a joint is receiving a new state ( R L Head)
    @param msg: message from action server where we get the information about the new state.
    ''' 
    def _state_callback(self, msg):
        for name_state, name_desi in zip(msg.joint_names, self._pr2.get_r_joints()):
            if name_desi == name_state:
                self.r_received = True
        for name_state, name_desi in zip(msg.joint_names, self._pr2.get_l_joints()):
            if name_desi == name_state:
                self.l_received = True
        for name_state, name_desi in zip(msg.joint_names, self._pr2.get_head_joints()):
            if name_desi == name_state:
               self.head_received = True
               
    '''
    @method _execute_callback: call back function to wait if the is nothing
    @param goal: ???
    @return: if rospy is off
    
    @note: can introduce up to 0.1 sec of latency
    ''' 
    def _execute_callback(self, goal):
        # Make sure we received arm state
        while not self.r_received or not self.l_received or not self.head_received:
            rospy.sleep(0.1)
            if rospy.is_shutdown():
                return
    
        # Create a new result
        result = TuckArmsResult()
    
        if(PRINT_ROS): rospy.loginfo(' START topic ')# -- Here are executed the different parts of the movement ')
    
           
        self._exec_move(self._pr2.get_speed(),self._pr2.get_r_states(),self._pr2.get_l_states(),self._pr2.get_head_states())

        if(PRINT_ROS): rospy.loginfo(' END  ' ) #-- Here are executed the different parts of the movement ')
    
        # Succeed or fail
        if self.success:
            if(PRINT_ROS): rospy.loginfo('-- SUCCESS --')
            self.action_server.set_succeeded(result)
            #rospy.signal_shutdown("Quitting")
        else:
            rospy.loginfo('-- FAILED --')
            #rospy.logerr("Tuck or untuck arms FAILED: Right value: %d. Left value: %d" % (result.tuck_left, result.tuck_right))
            self.action_server.set_aborted(result)
            
    '''
    @method _exec_move: Execute all the behaviors from the list of states

    The following parameters are LIST:
    @param speed: time to reach each state
    @param traj_r: states of right joints 
    @param traj_l: states of left joints 
    @param traj_head: states of head joints 
    '''    
    def _exec_move(self,speed,traj_r,traj_l,traj_head):
        for traj_via_r,traj_via_l, traj_via_head, speed_via in zip(traj_r,traj_l, traj_head, speed):
            self.move_duration = rospy.get_param('~move_duration', speed_via)
            self._go_RLHead(traj_via_r,traj_via_l,traj_via_head)
            
    '''
    @method _go_RLHead: Execute the behavior of one state

    @param traj_via_r: state of the right joints 
    @param traj_via_l: state of the left joints
    @param traj_via_head: state of the head joints
    '''    
    def _go_RLHead(self,traj_via_r,traj_via_l,traj_via_head):
        self._go('head', [traj_via_head])
        self._go('r', [traj_via_r])
        self._go('l', [traj_via_l])

    '''
    @method _go: append a new trajectory to the list of goals

    @param side: which side are we sending a new goal to reach
    @param positions: the positions of the states to reach
    @param wait: (always true for the moment)
    
    @todo: remove "wait" param
    '''    
    def _go(self, side, positions, wait = True):
        goal = JointTrajectoryGoal()
        if side == 'r':
            goal.trajectory.joint_names = [name for name in self._pr2.get_r_joints()]
        if side == 'l':
            goal.trajectory.joint_names = [name for name in self._pr2.get_l_joints()]
        if side == 'head':
            goal.trajectory.joint_names = [name for name in self._pr2.get_head_joints()]    
        
        goal.trajectory.points = []
        for p, count in zip(positions, range(0,len(positions)+1)):
            goal.trajectory.points.append(JointTrajectoryPoint( positions = p,
                                                              velocities = [],
                                                              accelerations = [],
                                                              time_from_start = rospy.Duration((count+1) * self.move_duration)))
          
        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01) 
        
        if wait:
            if side == 'head':
                self.head_joint_client.send_goal(goal)       
            if side == 'r':
                rospy.sleep(0.1)
                self.right_joint_client.send_goal(goal)
            if side == 'l':
                self.left_joint_client.send_goal_and_wait(goal)

        