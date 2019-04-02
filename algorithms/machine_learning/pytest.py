import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/python_bridge')
sys.path.append(dir_path + '/../../build/algorithms/machine_learning')

import pycheetah as cheetah
import numpy as np

for i in range(1):#(10000):
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
    #for j in range(18):
        #print out_config[j]
