'''
Created on July 22, 2014
Last change on Oct. 20, 2014

@author: Javier
'''

import threading

from Config import *

class Synchronizer:
    '''
    @Class: Manage the synchronization of the threads and keep track of them.
               
    The following attributes are LIST:
    @attribute self.goup_list: list of all the current threads running currently
    '''
    def __init__(self):
        if(DEBUG_MODE): print("init Synchronizer")
        self.goup_list = []

    '''
    @method create_thread: Called for each BML action, in order to create a new thread.
    @param some_Func: function launched in the thread
    @param args_of_some_Func: the arguments of the function launched in the thread
    @note: the thread is added to the list of current threads (self.goup_list)
    @todo: check if kikoo is still nedded
    '''
    def create_thread(self, some_Func, args_of_some_Func):
        # CHECK, it's working with out this hack now so I let it in case of there appear a bug with threads again
        #kikoo = threading.Thread(target=some_Func, args=args_of_some_Func)
        #kikoo.__init__()
        self.goup_list.append(threading.Thread(target=some_Func, args=args_of_some_Func))
        
    '''
    @method start_group_of_threads: Start simultaneously all the current threads in self.goup_list
    @method i.e. : Start simultaneously all the action inside <synchronize> =D </synchronize> from the BML file
    @note: The offset between the threads is manage inside each thread
    '''   
    def start_group_of_threads(self):
        for thread in self.goup_list:
            thread.start()  
            if(DEBUG_MODE): print('start new thread')
  
    '''
    @method delete_group_of_threads: Join point and delete all the current threads in self.goup_list
    @note: wait that all the threads finished there task, then terminate and delete them from self.goup_list.
    @note: Called after each block <synchronize> =D </synchronize> from the BML file
    '''         
    def delete_group_of_threads(self):
        for thread in self.goup_list:
            thread.join()
            
        for thread in self.goup_list:
            thread._Thread__stop()
        self.goup_list = []
        