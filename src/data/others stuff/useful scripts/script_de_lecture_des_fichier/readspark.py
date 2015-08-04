
import glob

class Main:

    def __init__(self):
        self.files_to_process = []
        
        self._joint_head_states = []
        self._joint_r_states = []
        self._joint_l_states = []
        self._speed = []

    def main(self):
        self.files_to_process = glob.glob("./files_to_process/*")
        #self.files_to_process.reverse()
        for file in self.files_to_process:
            print(file)
            self.read_file(file)

        with open("/home/javier/workspace_ros/javier_trajectory/src/Config_states.py", "a") as myfile:
            myfile.write("\n")
            myfile.write("# NEW Line \n")
            myfile.write("KIKOO_r = %s \n" % self._joint_r_states)
            myfile.write("KIKOO_l = %s \n" % self._joint_l_states)
            myfile.write("KIKOO_head = %s \n" % self._joint_head_states)
            myfile.write("KIKOO_speed = %s \n" % self._speed)
            myfile.write("\n")
            




    def read_file(self,file):
        joint_head_states_temp = []
        joint_r_states_temp = []
        joint_l_states_temp = []
        speed_temp = []
        
        with open(file, 'r') as courent_file:
            ''' read each line, one after the other '''
            content_line = courent_file.readlines()
            for line in content_line:
                words = line.split()

                if words[2] in ['q[16]','q[17]','q[18]','q[19]','q[20]','q[21]','q[22]']:
                    joint_r_states_temp.append(words[4])
                if words[2] in ['q[25]','q[26]','q[27]','q[28]','q[29]','q[30]','q[31]']:
                    joint_l_states_temp.append(words[4])   
                if words[2] in ['q[13]','q[14]']:
                    joint_head_states_temp.append(words[4])  
                # just to fill the speed with the value 1 sec 
                if words[2] in ['q[1]']:
                    self._speed.append('1.0') 
                    
                    
        self._joint_head_states.append(joint_head_states_temp)
        self._joint_r_states.append(joint_r_states_temp)
        self._joint_l_states.append(joint_l_states_temp) 

if __name__ == '__main__': 
    Main().main()
        
