#!/usr/bin/env python

import BML_reader

import rospy

from Config import *

PKG = 'javier_trajectory' # this package name
NAME = 'bml_server'

import roslib; roslib.load_manifest(PKG) 

# import the service
from javier_trajectory.srv import *

class Service_main:
    
    def __init__(self):
        if(DEBUG_MODE): print("init Main")
        self.my_reader = BML_reader.BML_reader()
        self.BML_string = ''

    def handle_bml_request(req):

        # example: self.my_reader.set_dom_from_file(bml_file, ("victor","table"),(("r_shoulder_pan_joint",45.0),("l_shoulder_pan_joint",45.0),("head_pan_joint",45.0)))
        
        #Load
        self.my_reader.set_dom_from_file(req.bml_file_name, req.list_of_speech_params ,req.list_of_adjustments)
        #Run
        self.my_reader.handleBML(self.my_reader.get_dom())
        
        return handle_bml_requestResponse(ok)

    def bml_service_server():
        rospy.init_node(NAME)
        s = rospy.Service('bml_service', Bml_srv_type, handle_bml_request)
    
        # spin() keeps Python from exiting until node is shutdown
        rospy.spin()

if __name__ == "__main__":
    Service_main.bml_service_server()
      
