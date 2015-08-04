'''
Created on 26 aout 2014

@author: Javier
'''

# Activate debugging mode
DEBUG_MODE = False
PRINT_ROS = False
PRINT_BML = True



# Path to data files : behaviors
if(DEBUG_MODE): print("init Config")
PATH = './src/data/behaviors/'
PATH_SPEECH = './src/data/speech/'

# TODO: have to be removed
BML_FILE_PATH = './src/data/BML.xml'

# add some delay to one or the other (but looks fine like this)
LAG_SPEECH = 0
LAG_BEHAVIOR = 0