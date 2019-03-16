import os 
dir_path = os.path.dirname(os.path.realpath(__file__))

import sys
sys.path.append(dir_path + '/python_bridge')
sys.path.append(dir_path + '/../../build/algorithms/machine_learning')

import pycheetah as cheetah
import numpy as np

for i in range(10000):
    in_jpos = cheetah.doubleArray(18)
    in_jvel = cheetah.doubleArray(18)

    for j in range(18):
        in_jpos[j] = 0.
        in_jvel[j] = 0.
    in_jpos[1] = -0.3
    in_jpos[2] = 1.
    in_jpos[3] = 0.3*np.sin(i*0.001)
    in_jvel[3] = 0.3*np.cos(i*0.001)
    cheetah.step(in_jpos, in_jvel)
