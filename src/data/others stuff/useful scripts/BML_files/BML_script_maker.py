
import glob


output_filename = 'attends'

joint_names = ["shoulder_pan", 
               "shoulder_lift",
               "upper_arm_roll",
               "elbow_flex", 
               "forearm_roll",
               "wrist_flex", 
               "wrist_roll" ]

joint_names_not_sym = ["pan","tilt"]

r = [[-0.023640, 0.790059, -0.165182, -2.121443, 0.010120, -0.222146, -0.172297],[-0.023640, 0.790059, -0.165182, -2.5, 0.010120, -0.222146, -0.172297],[-0.023640, 0.790059, -0.165182, -2.5, 0.010120, -0.222146, -0.172297]] 
l = [[0.099457, 0.462488, 0.257252, -1.461868, -0.165680, -0.567753, 0.016320],[0.099457, 0.462488, 0.257252, -1, -0.165680, -0.567753, 0.016320],[0.099457, 0.462488, 0.257252, -1, -0.165680, -0.567753, 0.016320]] 
head = [[-0.000316, 0.000838],[-0.000316, 0.000838],[-0.000316, 0.000838]] 
speed = [1.0,1.0,4.0] 


class Main:

    def __init__(self):
        self._joint_head_states = zip(*head)
        self._joint_r_states = zip(*r)
        self._joint_l_states = zip(*l)
        self._speed = speed

    def main(self):

        myfile = open('./output/'+output_filename+'.txt', "w")
        
        for joint_name,joint_r_states,joint_l_states in zip(joint_names,self._joint_r_states,self._joint_l_states):
            myfile.write('r_'+joint_name+'_joint')
            for r_state in joint_r_states:
                myfile.write(' %s' % r_state)
            myfile.write('\n')
            myfile.write('l_'+joint_name+'_joint')
            for l_state in joint_l_states:
                myfile.write(' %s' %l_state)
            myfile.write('\n')
        for joint_name, joint_head_states in zip(joint_names_not_sym,self._joint_head_states):
            myfile.write('head_%s_joint' % joint_name)
            for h_state in joint_head_states:
                myfile.write(' %s' % h_state)
            myfile.write('\n')
        myfile.write('speed')
        for speed_time in self._speed:
            myfile.write(' %s' % speed_time)
        myfile.write('\n')
        myfile.close()
        print('Done : %s' % output_filename)
        

if __name__ == '__main__': 
    Main().main()
        
