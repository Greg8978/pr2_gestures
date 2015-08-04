'''
Created on 21 juil. 2014
Last change on Oct. 22, 2014

@author: Javier
'''

import pyglet
import mad
import datetime

from Config import *

class Speech:
    
    '''
    @Class: Manage the execution of the speeches.

    @todo on read_text method: ACAPELA for text to speech handle
    @todo on play_mp3 method: "libGL error: failed to load driver: swrast"
    '''
    def __init__(self):
        if(DEBUG_MODE): print("init Speech")
        
    '''
    @method play_mp3: Play a mp3 file
    @param mp3_title: name of the mp3 file to play
    @todo: "libGL error: failed to load driver: swrast" <<== This error comes from here
    ''' 
    def play_mp3(self, mp3_title):
        #http://www.universal-soundbank.com/ <<== To get some sound tracks 

        full_path = PATH_SPEECH + mp3_title + '.mp3'
        
        song = pyglet.media.load(full_path , streaming=False)
        song.play()

        #Get length of the track in milliseconds
        mf = mad.MadFile(full_path)
        track_length_in_milliseconds = mf.total_time()

        # Add 10% extra time or a minimum of 1s before killing the thread
        track_length_in_seconds = track_length_in_milliseconds*0.0011
        if(track_length_in_seconds<1):
            track_length_in_seconds = 1.0
        
        # Schedule the end of the thread when the track is finish
        pyglet.clock.schedule_once(self._exit_callback, track_length_in_seconds)
        pyglet.app.run()
        
        if(PRINT_BML): print("    <speech>%s</speech>" % (mp3_title))

    '''
    @method _exit_callback: Callback method to end the thread when the mp3 file has finish playing
    @param time_in_seconds: time in seconds to wait before calling this method (here it will be length of the mp3 track)
    @note: used in "play_mp3" method
    ''' 
    def _exit_callback(self, time_in_seconds):
        pyglet.app.exit()
 
    '''
    @method read_text: Text to speech method using ACAPELA
    @param text: text to be read
    @todo: ACAPELA IS NOT MANAGE YET 
    ''' 
    def read_text(self, text):
        if(PRINT_BML): print("    <speech>%s</speech>" % (text))
