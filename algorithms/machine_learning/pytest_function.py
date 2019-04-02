import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/python_bridge')
sys.path.append(dir_path + '/../../build/algorithms/machine_learning')

import pycheetah as cheetah
import numpy as np

T_sampling = 10

def reward_function(state):
    z_com = state[5]
    z_com_d = 0.3
    reward = -(z_com-z_com_d)**2
    return reward

def run_sim():
    for i in range(T_sampling):#(10000):
        print i
        in_jpos = cheetah.doubleArray(12)
        in_jvel = cheetah.doubleArray(12)

        out_config = cheetah.doubleArray(18)
        out_config_vel = cheetah.doubleArray(18)

        for j in range(12):
            in_jpos[j] = 0.
            in_jvel[j] = 0.

        in_jpos[0] = -0.03
        in_jpos[1] = -0.79
        in_jpos[2] = 1.715

        in_jpos[3] = 0.03
        in_jpos[4] = -0.72
        in_jpos[5] = 1.715

        in_jpos[6] = -0.03
        in_jpos[7] = -0.79
        in_jpos[8] = 1.715

        in_jpos[9] = 0.03
        in_jpos[10] = -0.72
        in_jpos[11] = 1.715



        in_jpos[2] += 0.3*np.sin(i*0.001)
        in_jvel[2] = 0.3*np.cos(i*0.001)

        in_jpos[5] += 0.3*np.sin(i*0.001)
        in_jvel[5] = 0.3*np.cos(i*0.001)

        in_jpos[8] += 0.3*np.sin(i*0.001)
        in_jvel[8] = 0.3*np.cos(i*0.001)

        in_jpos[11] += 0.3*np.sin(i*0.001)
        in_jvel[11] = 0.3*np.cos(i*0.001)

        cheetah.step(in_jpos, in_jvel, out_config, out_config_vel)

    state = np.zeros(36)

    for i in range(18):
        state[i] = out_config[i]
        state[i+18] = out_config_vel[i]

    reward = reward_function(state)

    return state, reward
        #for j in range(18):
            #print out_config[j]

if __name__ == "__main__":
#    cheetah_reinforcement_learning()
    state, reward = run_sim()
    print state
    print reward
