'''
Created on 17 juil. 2014
Last change on Oct. 23, 2014

@author: Javier
'''
import xml.dom.minidom
import BML_controler
import Synchronizer

from Config import *

class BML_reader:
    '''
    @Class: Handle the reading of BML (XML Parser bml like)
               
    @attribute self.dom: object to parse BML
    @attribute self._my_BML_controler: from where all the action will be handled and executed.
    @attribute self._sync: from where the synchronization is manage.
    '''
    def __init__(self):
        if(DEBUG_MODE): print("init BML_reader")
        self._dom = None
        self._my_BML_controler = BML_controler.BML_controler()
        self._sync = Synchronizer.Synchronizer()
        
        self._speech_param = []
        self._behavior_param = []
    '''
    @method set_dom_from_file: set the document that represents the BML file
    @param BML_file_to_read: path to the BML file parsed
    '''  
    def set_dom_from_file(self, BML_file_to_read, speech_params, behavior_params ):
        self._speech_param = []
        self._speech_param = speech_params
        self._behavior_param = []
        self._behavior_param = behavior_params
        self._dom = xml.dom.minidom.parse(BML_file_to_read)

    '''
    @method set_dom_from_string: set the document that represents the string
    @param BML_string_to_read: path to the BML string parsed
    '''  
    def set_dom_from_string(self, BML_string_to_read, speech_params, behavior_params ):
        self._speech_param = []
        self._speech_param = speech_params
        self._behavior_param = []
        self._behavior_param = behavior_params
        self._dom = xml.dom.minidom.parseString(BML_string_to_read)

    '''
    @method get_dom: Getter for the document that is represented by BML
    @return self._dom: returns the document representing the BML information contained
    '''  
    def get_dom(self):
        return self._dom

    '''
    @method _getText: Getter the text between the the BML tags
    @param nodelist:
    @return ''.join(rc): returns the text read between the BML tags
    '''  
    def _getText(self, nodelist):
        rc = []
        for node in nodelist:
            if node.nodeType == node.TEXT_NODE:
                rc.append(node.data)
        return ''.join(rc)

    '''
    @method _handleSpeeches: Handle all speeches inside a <synchronize> block
    @param speeches: list of all the speeches to execute
    '''  
    def _handleSpeeches(self, speeches):
        for speech in speeches:
            self._handleSpeech(speech)

    '''
    @method _handleSpeech: Handle a speech, executing it inside a new thread for synchronization purpose.
    @param speech: speech to be executed
    
    @note: two BML parameters are handle inside the speech tag : 'offset' and 'type'
    '''  
    def _handleSpeech(self, speech):
        speech_offset = speech.getAttribute('offset')
        speech_type = speech.getAttribute('type')
        speech_speech = self._getText(speech.childNodes)
        
        string_of_params = speech.getAttribute('param')
        if speech_type == "text":
            speech_speech = self._replace_params_in_speech(string_of_params, speech_speech)
            
        self._sync.create_thread(self._my_BML_controler.perform_speech, (speech_speech, self._string_to_float(speech_offset), speech_type) ) 
  
    '''
    @method _handleBehaviors: Handle all behaviors inside a <synchronize> block
    @param behaviors: list of all the behaviors to execute
    '''  
    def _handleBehaviors(self, behaviors):
        for behavior in behaviors:
            self._handleBehavior(behavior)     
             
    '''
    @method _handleBehavior: Handle a behavior, executing it inside a new thread for synchronization purpose.
    @param behavior: behavior to be executed
    
    @note: one BML parameter is handle inside the behavior tag : 'offset'
    @note: 
    '''  
    def _handleBehavior(self, behavior):
        behavior_offset = behavior.getAttribute('offset')
        behavior_behavior = self._getText(behavior.childNodes)
        self._sync.create_thread(self._my_BML_controler.perform_behavior,(behavior_behavior, self._string_to_float(behavior_offset), self._behavior_param))
 
    '''
    @method _handleSynchronizes: Handle all synchronize blocks inside a <bml> block
    @param synchronizes: list of all the synchronize block to handle
    '''  
    def _handleSynchronizes(self, synchronizes):
        for synchronize in synchronizes:
            self._handleSynchronize(synchronize)      

    '''
    @method _handleSynchronize: Handle a synchronize block.
    @param synchronize: synchronize block to handle
    
    @note: Start all the actions inside the synchronize block and wait that all have finished before moving forward
    '''  
    def _handleSynchronize(self, synchronize):
        if(PRINT_BML): print("  <synchronize>")
        self._handleBehaviors(synchronize.getElementsByTagName("behavior"))
        self._handleSpeeches(synchronize.getElementsByTagName("speech"))
        
        self._sync.start_group_of_threads()
        self._sync.delete_group_of_threads()
        if(PRINT_BML): print("  </synchronize>") 
        if(PRINT_BML): print("")

    '''
    @method handleBML: Handle a bml block.
    @param bml: bml block, i.e. the document with the BML information inside
    '''      
    def handleBML(self, bml):
        if(PRINT_BML): print("")
        if(PRINT_BML): print("<bml>")
        if(PRINT_BML): print("")
        
        synchronizes = bml.getElementsByTagName("synchronize")
        self._handleSynchronizes(synchronizes)
        
        if(PRINT_BML): print("</bml>") 
        if(PRINT_BML): print("")
  
    '''
    @method _string_to_float: String to float method
    
    @note: if string is empty i.e. if we don't specify it, we set it by default to 0 
    
    @return string: string that will be transformed in float
    '''      
    def _string_to_float(self, string):
        if(string == ""): string = "0"
        return float(string)


    '''
    @method _replace_params_in_speech: 
    @param string_of_params:
    @param speech:   
    '''      
    def _replace_params_in_speech(self, string_of_params, speech):
        list_of_params = string_of_params.split()
        for tag, value in zip(list_of_params, self._speech_param):
            speech = speech.replace(tag, value)
        return speech

    '''
    @method _string_to_float: String to float method
    @param list_of_param:
    '''      
    def _add_params_to_behavior(self, list_of_param):
        pass